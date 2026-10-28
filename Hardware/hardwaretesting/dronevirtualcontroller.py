import sys
from pathlib import Path

import pygame
from hardwaretodrone import Drone

IMAGE_DIR = Path(__file__).parent / "dronevirtualcontrollerimages"

BUTTONS = [
    ("upbutton.png", "up"),
    ("downbutton.png", "down"),
    ("forwardbutton.png", "forward"),
    ("turningbutton.png", "turn_right"),
]

HOVER_TINT = (25, 25, 25)    # added to the image's colors
PRESS_TINT = (60, 60, 60)    # subtracted from the image's colors


class UIButton:
    def __init__(self, image_file, command):
        self.command = command
        self.image = pygame.image.load(IMAGE_DIR / image_file).convert_alpha()
        mask = pygame.mask.from_surface(self.image)
        rects = mask.get_bounding_rects()
        self.rect = rects[0].unionall(rects[1:])

        self.hover_image = self.image.copy()
        self.hover_image.fill(HOVER_TINT, special_flags=pygame.BLEND_RGB_ADD)
        self.pressed_image = self.image.copy()
        self.pressed_image.fill(PRESS_TINT, special_flags=pygame.BLEND_RGB_SUB)

        self.hovered = False
        self.pressed = False

    def hit(self, pos):
        return self.rect.collidepoint(pos)

    def draw(self, surface):
        if self.pressed:
            surface.blit(self.pressed_image, (0, 0))
        elif self.hovered:
            surface.blit(self.hover_image, (0, 0))
        else:
            surface.blit(self.image, (0, 0))


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Fly Brains Virtual Controller")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 28)

    buttons = [UIButton(f, c) for f, c in BUTTONS]
    active = None

    dry_run = "--dry-run" in sys.argv
    with Drone(strength=30, dry_run=dry_run) as drone:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    active = next((b for b in buttons if b.hit(event.pos)), None)
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    active = None

            mouse = pygame.mouse.get_pos()
            for b in buttons:
                b.hovered = b.hit(mouse)
                b.pressed = b is active and b.hovered

            held = next((b for b in buttons if b.pressed), None)
            if held:
                drone.command(held.command)
            else:
                drone.hover()

            screen.fill((255, 255, 255))
            for b in buttons:
                b.draw(screen)
            status = f"{held.command}" if held else "hover"
            screen.blit(font.render(status, True, (0, 0, 0)), (10, 560))
            pygame.display.flip()
            clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    print("[DRONE CONTROLLER] Cheeck for if the drone is currently active.")

    response = input("Y/N").lower()
    if(response == "y"):
        print("[DRONE CONTROLLER] Run up down before following.")


    #yeah i dont have a N exit case lol who cares its just for testing
    main()