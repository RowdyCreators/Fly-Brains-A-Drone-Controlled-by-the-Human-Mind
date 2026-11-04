# convert_dataset.py

import pickle
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import welch


# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

INPUT_FILE = Path("data/raw/epoched_train.pkl")
OUTPUT_FILE = Path("data/processed/eeg_motor_imagery_features.csv")

SAMPLING_RATE = 250

CHANNELS = ["C3", "Cz", "C4"]

BANDS = {
    "alpha": (8, 13),
    "beta": (13, 30),
    "gamma": (30, 45),
}

LABEL_MAP = {
    0: "LEFT_HAND",
    1: "RIGHT_HAND",
}


# ---------------------------------------------------------
# Band-power calculation
# ---------------------------------------------------------

def calculate_band_power(signal, sampling_rate, low_freq, high_freq):
    """
    Calculate power contained within a frequency range
    using Welch's power spectral density estimate.
    """

    signal = np.asarray(signal, dtype=float)

    frequencies, psd = welch(
        signal,
        fs=sampling_rate,
        nperseg=min(256, len(signal)),
    )

    mask = (
        (frequencies >= low_freq)
        & (frequencies < high_freq)
    )

    if not np.any(mask):
        return np.nan

    return np.trapezoid(
        psd[mask],
        frequencies[mask]
    )


def extract_channel_features(signal):
    """
    Convert one channel's raw EEG time-series into:

    [alpha, beta, gamma]
    """

    features = {}

    for band_name, (low, high) in BANDS.items():
        features[band_name] = calculate_band_power(
            signal,
            SAMPLING_RATE,
            low,
            high,
        )

    return features


# ---------------------------------------------------------
# Data helpers
# ---------------------------------------------------------

def convert_signal_to_array(value):
    """
    Ensures that a channel value becomes a NumPy array.

    Supports:
        list
        tuple
        numpy.ndarray
        pandas.Series
    """

    if isinstance(value, pd.Series):
        value = value.to_numpy()

    return np.asarray(value, dtype=float).flatten()


def validate_trial(row):
    """
    Check that all required channels exist and contain data.
    """

    for channel in CHANNELS:

        if channel not in row:
            return False

        signal = convert_signal_to_array(row[channel])

        if len(signal) == 0:
            return False

        if np.isnan(signal).any():
            return False

    return True


# ---------------------------------------------------------
# Main conversion
# ---------------------------------------------------------

def convert_dataset(df):

    converted_rows = []

    for trial_number, (_, row) in enumerate(df.iterrows(), start=1):

        if not validate_trial(row):
            print(f"Skipping invalid trial {trial_number}")
            continue

        output = {
            "trial": trial_number,
        }

        # -------------------------------------------------
        # Preserve patient ID if available
        # -------------------------------------------------

        if "patient_id" in df.columns:
            output["patient_id"] = row["patient_id"]

        # -------------------------------------------------
        # Convert label
        # -------------------------------------------------

        raw_label = row.get("event_type", None)

        output["label"] = LABEL_MAP.get(
            raw_label,
            str(raw_label)
        )

        # -------------------------------------------------
        # Extract alpha/beta/gamma for all 3 channels
        # -------------------------------------------------

        for channel in CHANNELS:

            signal = convert_signal_to_array(
                row[channel]
            )

            band_features = extract_channel_features(
                signal
            )

            output[f"{channel}_alpha"] = band_features["alpha"]
            output[f"{channel}_beta"] = band_features["beta"]
            output[f"{channel}_gamma"] = band_features["gamma"]

        converted_rows.append(output)

    return pd.DataFrame(converted_rows)


# ---------------------------------------------------------
# Run
# ---------------------------------------------------------

def main():

    print(f"Loading: {INPUT_FILE}")

    with open(INPUT_FILE, "rb") as file:
        data = pickle.load(file)

    print("Loaded object type:", type(data))

    if not isinstance(data, pd.DataFrame):
        raise TypeError(
            "Expected the pickle file to contain a pandas DataFrame. "
            f"Received: {type(data)}"
        )

    print("Original shape:", data.shape)
    print("Columns:")
    print(data.columns.tolist())

    # Confirm required channels exist
    missing_channels = [
        channel
        for channel in CHANNELS
        if channel not in data.columns
    ]

    if missing_channels:
        raise ValueError(
            f"Missing expected channels: {missing_channels}\n"
            "Run print(data.columns.tolist()) to inspect the dataset."
        )

    # Convert
    converted = convert_dataset(data)

    # Create output directory if needed
    OUTPUT_FILE.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    converted.to_csv(
        OUTPUT_FILE,
        index=False
    )

    print()
    print("Conversion finished.")
    print("New shape:", converted.shape)
    print()
    print(converted.head())
    print()
    print(f"Saved to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()