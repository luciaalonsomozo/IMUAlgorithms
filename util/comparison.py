import pandas as pd
import matplotlib.pyplot as plt

csv_filename = "files.csv"  

with open(csv_filename, "r") as file:
    first_line = file.readline().strip() 

file_paths = first_line.split(",")

files = {
    "Complementary Filter": file_paths[5],
    "Euler Direct": file_paths[6],
    "Kalman Filter": file_paths[10],
    "Fusion Simple": file_paths[9],
    "Fusion Advanced": file_paths[8],
    "Fusion Advanced Calibration":file_paths[7],
}

colors = {
    "Complementary Filter": "red",
    "Euler Direct": "blue",
    "Kalman Filter": "green",
    "Fusion Simple": "orange",
    "Fusion Advanced": "magenta",
    "Fusion Advanced Calibration": "cyan",
}

data = {}

for method, archivo in files.items():
    try:
        df = pd.read_csv(archivo)
        data[method] = df
    except FileNotFoundError:
        print(f"File couldn't be found: {archivo}")

fig, axs = plt.subplots(2, 1, figsize=(10, 8))
for method, df in data.items():
    axs[0].plot(df["Time"], df["Phi (degrees)"], label=method, color=colors[method])

axs[0].set_title("Time vs Roll")
axs[0].set_xlabel("Time (s)")
axs[0].set_ylabel("Roll (degrees)")
axs[0].grid(True)

for method, df in data.items():
    axs[1].plot(df["Time"], df["Theta (degrees)"], label=method, color=colors[method])

axs[1].set_title("Time vs Pitch")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Pitch (degrees)")
axs[1].grid(True)

plt.tight_layout()
plt.show()
