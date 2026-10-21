/*
Note from samuel: this plays no significance int he final product, it's instead for the building process to test with my multimeter.
i had claude writethis up for me.
 * dac_sweep.c — step an MCP4728 output from 0V to VDD so you can watch it
 *               on a multimeter.
 *
 * Reads the bus, DAC address, supply voltage and axis names from hardware.yaml.
 * Talks to the chip with raw I2C writes through /dev/i2c-N (no extra libs).
 *
 * Build:  gcc -O2 -Wall -o dac_sweep dac_sweep.c
 * Run:    ./dac_sweep                 sweep channel A (default)
 *         ./dac_sweep -ch B           sweep channel B
 *         ./dac_sweep -ch all         sweep A, B, C, D (pauses to move probe)
 *         ./dac_sweep -step 0.25 -hold 5 -c path/to/hardware.yaml
 *
 * Ctrl-C at any time → every channel is driven to VDD/2 (stick centered).
 */


#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
 
/* ───────────────────────── config ───────────────────────── */
 
typedef struct {
    int    i2c_bus;          /* pi.i2c_bus               */
    int    dac_addr;         /* control.dac.i2c_address  */
    double vdd;              /* pi.logic_voltage (Vref = VDD) */
    char   axis_name[4][16]; /* control.axes.<name>: { channel: X } */
} config_t;
 
static void config_defaults(config_t *c) {
    c->i2c_bus  = 1;
    c->dac_addr = 0x60;
    c->vdd      = 3.3;
    for (int i = 0; i < 4; i++) strcpy(c->axis_name[i], "?");
}
 
/* Minimal YAML reader: only understands "key: value" lines nested by
 * indentation, which is all hardware.yaml uses. Not a general YAML parser. */
static char *trim(char *s) {
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\n' || e[-1] == '\r')) e--;
    *e = '\0';
    return s;
}
 
static int config_load(const char *path, config_t *c) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
 
    struct { int indent; char key[32]; } stack[8];
    int depth = 0;
    char line[256];
 
    while (fgets(line, sizeof line, f)) {
        char *hash = strchr(line, '#');           /* drop comments */
        if (hash) *hash = '\0';
 
        int indent = 0;
        while (line[indent] == ' ') indent++;
        char *body = trim(line);
        if (*body == '\0') continue;
 
        char *colon = strchr(body, ':');
        if (!colon) continue;
        *colon = '\0';
        char *key = trim(body);
        char *val = trim(colon + 1);
 
        while (depth > 0 && stack[depth - 1].indent >= indent) depth--;   /* pop to parent */
 
        /* build dotted path of the parents, e.g. "control.dac" */
        char parent[128] = "";
        for (int i = 0; i < depth; i++) {
            if (i) strcat(parent, ".");
            strcat(parent, stack[i].key);
        }
 
        if (*val == '\0') {                       /* section header → push */
            if (depth < 8) {
                stack[depth].indent = indent;
                snprintf(stack[depth].key, sizeof stack[depth].key, "%s", key);
                depth++;
            }
            continue;
        }
 
        if (!strcmp(parent, "pi") && !strcmp(key, "i2c_bus"))
            c->i2c_bus = (int)strtol(val, NULL, 0);
        else if (!strcmp(parent, "pi") && !strcmp(key, "logic_voltage"))
            c->vdd = strtod(val, NULL);
        else if (!strcmp(parent, "control.dac") && !strcmp(key, "i2c_address"))
            c->dac_addr = (int)strtol(val, NULL, 0);           /* handles 0x60 */
        else if (!strcmp(parent, "control.axes")) {
            /* val looks like: { channel: A, min_v: null, ... } */
            char *ch = strstr(val, "channel:");
            if (ch) {
                ch = trim(ch + 8);
                int idx = *ch - 'A';
                if (idx >= 0 && idx < 4)
                    snprintf(c->axis_name[idx], sizeof c->axis_name[idx], "%s", key);
            }
        }
    }
    fclose(f);
    return 0;
}
 
/* ───────────────────────── MCP4728 ───────────────────────── */
 
static int g_fd = -1;
static volatile sig_atomic_t g_stop = 0;
 
static void on_sigint(int sig) { (void)sig; g_stop = 1; }
 
/* Multi-Write command (0b01000 CC U): updates the DAC register only.
 * Unlike Single Write it does NOT touch EEPROM, so it's safe to spam.
 *   byte0: 0 1 0 0 0 DAC1 DAC0 UDAC   (UDAC=0 → output updates immediately)
 *   byte1: VREF PD1 PD0 Gx D11 D10 D9 D8  (VREF=0 → VDD, PD=00 → on, Gx=0)
 *   byte2: D7..D0
 */
static int dac_write(int ch, uint16_t code) {
    if (code > 4095) code = 4095;
    uint8_t buf[3] = {
        (uint8_t)(0x40 | (ch << 1)),
        (uint8_t)((code >> 8) & 0x0F),
        (uint8_t)(code & 0xFF),
    };
    return write(g_fd, buf, 3) == 3 ? 0 : -1;
}
 
/* Read back all 24 status bytes (6 per channel: 3 live register + 3 EEPROM)
 * and return the live code for one channel, or -1 on error. */
static int dac_readback(int ch) {
    uint8_t buf[24];
    if (read(g_fd, buf, 24) != 24) return -1;
    uint8_t *r = &buf[ch * 6];
    return ((r[1] & 0x0F) << 8) | r[2];
}
 
static uint16_t volts_to_code(double v, double vdd) {
    if (v < 0) v = 0;
    if (v > vdd) v = vdd;
    return (uint16_t)(v / vdd * 4095.0 + 0.5);
}
 
static void center_all(double vdd) {
    for (int ch = 0; ch < 4; ch++) dac_write(ch, volts_to_code(vdd / 2, vdd));
}
 
/* ───────────────────────── sweep ───────────────────────── */
 
static int sweep_channel(int ch, const config_t *c, double step, int hold) {
    printf("\n── Channel %c (%s) ── probe on V%c, black probe on GND\n",
           'A' + ch, c->axis_name[ch], 'A' + ch);
    printf("  target V   code   readback   (write your multimeter value next to each)\n");
 
    for (double v = 0.0; v <= c->vdd + 1e-9 && !g_stop; v += step) {
        uint16_t code = volts_to_code(v, c->vdd);
        if (dac_write(ch, code) < 0) {
            fprintf(stderr, "  write failed: %s\n", strerror(errno));
            return -1;
        }
        int rb = dac_readback(ch);
        printf("  %6.3f V   %4u   %4d %s\n", v, code, rb,
               rb == code ? "" : "  <-- MISMATCH");
        fflush(stdout);
        for (int s = 0; s < hold && !g_stop; s++) sleep(1);
    }
 
    /* finish on full scale so the last reading is the true max */
    if (!g_stop) {
        dac_write(ch, 4095);
        printf("  %6.3f V   4095   %4d   (full scale)\n", c->vdd, dac_readback(ch));
        for (int s = 0; s < hold && !g_stop; s++) sleep(1);
    }
    return 0;
}
 
int main(int argc, char **argv) {
    const char *cfg_path = "hardware.yaml";
    const char *ch_arg   = "all";
    double step = 0.5;
    int    hold = 3;
 
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "-c")    && i + 1 < argc) cfg_path = argv[++i];
        else if (!strcmp(argv[i], "-ch")   && i + 1 < argc) ch_arg   = argv[++i];
        else if (!strcmp(argv[i], "-step") && i + 1 < argc) step     = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "-hold") && i + 1 < argc) hold     = atoi(argv[++i]);
        else {
            fprintf(stderr, "usage: %s [-ch A|B|C|D|all] [-step volts] [-hold sec] [-c hardware.yaml]\n", argv[0]);
            return 1;
        }
    }
    if (step <= 0) step = 0.5;
 
    int single_ch = -1;                           /* -1 means "all" */
    if (strcmp(ch_arg, "all")) {
        single_ch = ch_arg[0] - 'A';
        if (single_ch < 0 || single_ch > 3 || ch_arg[1]) {
            fprintf(stderr, "channel must be A, B, C, D or all\n");
            return 1;
        }
    }
 
    config_t cfg;
    config_defaults(&cfg);
    if (config_load(cfg_path, &cfg) < 0)
        fprintf(stderr, "warning: couldn't read %s, using defaults\n", cfg_path);
 
    printf("bus /dev/i2c-%d  addr 0x%02X  VDD %.2fV  step %.2fV  hold %ds\n",
           cfg.i2c_bus, cfg.dac_addr, cfg.vdd, step, hold);
 
    char dev[32];
    snprintf(dev, sizeof dev, "/dev/i2c-%d", cfg.i2c_bus);
    g_fd = open(dev, O_RDWR);
    if (g_fd < 0) { perror(dev); return 1; }
    if (ioctl(g_fd, I2C_SLAVE, cfg.dac_addr) < 0) { perror("I2C_SLAVE"); return 1; }
 
    /* probe: if the readback fails, the chip isn't answering */
    if (dac_readback(0) < 0) {
        fprintf(stderr, "no response at 0x%02X — check wiring / run i2cdetect -y %d\n",
                cfg.dac_addr, cfg.i2c_bus);
        return 1;
    }
 
    signal(SIGINT, on_sigint);
 
    /* start from a known state: every channel centered */
    center_all(cfg.vdd);
 
    int first = (single_ch < 0) ? 0 : single_ch;
    int last  = (single_ch < 0) ? 3 : single_ch;
 
    for (int ch = first; ch <= last && !g_stop; ch++) {
        printf("\nPut the red probe on V%c (%s), then press Enter...",
               'A' + ch, cfg.axis_name[ch]);
        fflush(stdout);
        int k; while ((k = getchar()) != '\n' && k != EOF) {}
        if (g_stop) break;
 
        if (sweep_channel(ch, &cfg, step, hold) < 0) break;
        dac_write(ch, volts_to_code(cfg.vdd / 2, cfg.vdd));   /* re-center when done */
    }
 
    center_all(cfg.vdd);
    printf("\n%s — all channels set to %.2fV (center)\n",
           g_stop ? "Interrupted" : "Done", cfg.vdd / 2);
    close(g_fd);
    return 0;
}