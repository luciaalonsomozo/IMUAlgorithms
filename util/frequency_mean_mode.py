import pandas as pd
from scipy.stats import mode

csv_filename = "files.csv"
with open(csv_filename, "r") as file:
    first_line = file.readline().strip() 
    
file_paths = first_line.split(",")
data_file = file_paths[4]

df = pd.read_csv(data_file, header=None)
df[0] = pd.to_numeric(df[0], errors='coerce')

time = df.iloc[:, 0]
differences = time.diff().dropna()

frequency = 1 / differences.mean()

print(f"Mean frequency: {frequency:.2f} Hz")

mode_result = mode(differences, keepdims=False)
mode = mode_result.mode
frequency_mode = 1 / mode if mode != 0 else None

if frequency_mode:
    print(f"Mode frequency: {frequency_mode:.2f} Hz")