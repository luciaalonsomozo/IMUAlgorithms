import pandas as pd
import numpy as np

# Load paths from CSV file
csv_filename = "files.csv"
with open(csv_filename, "r") as file:
    first_line = file.readline().strip()
file_paths = first_line.split(",")

# Associate methods with file paths
files = {
    "Complementary Filter": file_paths[5],
    "Euler Direct": file_paths[6],
    "Kalman Filter": file_paths[10],
    "Fusion Simple": file_paths[9],
    "Fusion Advanced": file_paths[8],
    "Fusion Advanced Calibration": file_paths[7],
}

# Load data
data = {}
for method, archivo in files.items():
    try:
        df = pd.read_csv(archivo)
        data[method] = df
    except FileNotFoundError:
        print(f"File couldn't be found: {archivo}")

# Compute errors with respect to Euler Direct
errors = {}

# Reference data
euler_df = data["Euler Direct"]
euler_roll = euler_df["Phi (degrees)"].values
euler_pitch = euler_df["Theta (degrees)"].values
time_ref = euler_df["Time"].values

for method, df in data.items():
    if method == "Euler Direct":
        continue

    # Interpolate data to match Euler Direct timestamps
    interp_roll = np.interp(time_ref, df["Time"], df["Phi (degrees)"])
    interp_pitch = np.interp(time_ref, df["Time"], df["Theta (degrees)"])

    # Compute absolute error
    roll_error = np.abs(interp_roll - euler_roll)
    pitch_error = np.abs(interp_pitch - euler_pitch)

    # Compute mean errors
    mean_roll_error = np.mean(roll_error)
    mean_pitch_error = np.mean(pitch_error)
    total_error = (mean_roll_error + mean_pitch_error) / 2

    errors[method] = {
        "Roll Error": mean_roll_error,
        "Pitch Error": mean_pitch_error,
        "Total Error": total_error
    }

# Print results
print("\nError Comparison (vs Euler Direct):")
for method, err in errors.items():
    print(f"{method}:")
    print(f"  Mean Roll Error  = {err['Roll Error']:.3f}°")
    print(f"  Mean Pitch Error = {err['Pitch Error']:.3f}°")
    print(f"  Total Mean Error = {err['Total Error']:.3f}°\n")
