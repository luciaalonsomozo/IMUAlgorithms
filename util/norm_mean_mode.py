import numpy as np
import pandas as pd
from scipy.stats import mode

# Load file paths
csv_filename = "files.csv"
with open(csv_filename, "r") as file:
    first_line = file.readline().strip()
    
file_paths = first_line.split(",")
data_file = file_paths[1]

# Load the data 
data = pd.read_csv(data_file, header=None).values

print(data_file)
print(data)

# Custom "norm" calculation: sqrt(first^2 + second^2 + third^2)
custom_norms = np.sqrt(data[:, 0]**2 + data[:, 1]**2 + data[:, 2]**2)

# Group size
group_size = 1000

# Number of full groups
num_groups = len(custom_norms) // group_size

# Lists to store means and modes
mean_norms = []
mode_norms = []

# Calculate mean and mode for each group
for i in range(num_groups):
    start = i * group_size
    end = start + group_size
    group = custom_norms[start:end]
    
    mean_norm = np.mean(group)
    mean_norms.append(mean_norm)

    mode_norm = mode(group, keepdims=False).mode  # keepdims=False to get scalar
    mode_norms.append(mode_norm)

if len(custom_norms) % group_size != 0:
    group = custom_norms[num_groups * group_size:]
    
    mean_norm = np.mean(group)
    mean_norms.append(mean_norm)

    mode_norm = mode(group, keepdims=False).mode
    mode_norms.append(mode_norm)

# Print results
for idx, (mean_value, mode_value) in enumerate(zip(mean_norms, mode_norms)):
    print(f"Group {idx+1}: Mean norm = {mean_value:.5f}, Mode norm = {mode_value:.5f}")

print("Mean norms:", mean_norms)
print("Mode norms:", mode_norms)
