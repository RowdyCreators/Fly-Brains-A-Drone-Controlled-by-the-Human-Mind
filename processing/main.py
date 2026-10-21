"""Processing entry point for the brain-signal controller."""

from signals.signal import FlyBrian


def main():
    """Collect a short batch of sample readings and print them."""
    brain = FlyBrian(sample_rate=0.01)
    readings = brain.main(number_of_samples=10)
    print(readings)
    return readings


if __name__ == "__main__":
    main()

