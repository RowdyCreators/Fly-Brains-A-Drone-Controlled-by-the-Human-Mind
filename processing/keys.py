"""Single-key input without Enter (for driving the simulator while flying).

Keys toggle a sim state: c = clench, f = focus, e = eyes_closed,
space or r = back to rest, q = quit. Works on Linux/macOS (termios) and Windows (msvcrt).
"""

import sys
import threading
import time

KEYMAP = {"c": "clench", "f": "focus", "e": "eyes_closed", " ": "rest", "r": "rest"}
HELP = "sim keys: c=clench  f=focus  e=eyes_closed  (press again or space = rest)  q=quit"


def _reader():
    """Return (get_key(timeout) -> str|None, restore())."""
    try:
        import msvcrt

        def get_key(timeout):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if msvcrt.kbhit():
                    return msvcrt.getwch()
                time.sleep(0.02)
            return None
        return get_key, lambda: None
    except ImportError:
        import select
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        def get_key(timeout):
            r, _, _ = select.select([sys.stdin], [], [], timeout)
            return sys.stdin.read(1) if r else None
        return get_key, lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old)


class SimKeyboard:
    """Background thread that sets brain.state from key presses.
    `quit` is set when q is pressed."""

    def __init__(self, brain):
        self.brain = brain
        self.quit = threading.Event()
        self._stop = threading.Event()
        self._get_key, self._restore = _reader()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            k = self._get_key(0.1)
            if k is None:
                continue
            k = k.lower()
            if k == "q":
                self.quit.set()
            elif k in KEYMAP:
                want = KEYMAP[k]
                self.brain.state = "rest" if self.brain.state == want else want

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=0.5)
        self._restore()
