import pandas as pd

# To calibrate the gyroscope, the data should be in a CSV file with the following format:
# GyrX,GyrY,GyrZ.
# To get good results, the data should be collected while the device is stationary.

csv_filename = "files.csv"  

with open(csv_filename, "r") as file:
    first_line = file.readline().strip()  

file_paths = first_line.split(",")
file = file_paths[3]

df = pd.read_csv(file, header=None, names=["GyrX", "GyrY", "GyrZ"])

mean_gyrX = df["GyrX"].mean()
mean_gyrY = df["GyrY"].mean()
mean_gyrZ = df["GyrZ"].mean()

print([mean_gyrX, mean_gyrY, mean_gyrZ])
