import csv
import math
from panda3d.core import LQuaternionf, Vec3

csv_filename = "carpetas_datos.csv"  
with open(csv_filename, "r") as file:
    first_line = file.readline().strip()


file_paths = first_line.split(",")

input_file = file_paths[3]
output_file = file_paths[6]

with open(output_file, mode='w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(["Time", "Phi (degrees)", "Theta (degrees)" , "Yaw (degrees)"])

    with open(input_file, mode='r', newline='') as f:
        reader = csv.reader(f)
        next(reader)

        for row in reader:
            time = float(row[0])
            qw = float(row[1])
            qx = float(row[2])
            qy = float(row[3])
            qz = float(row[4])
            
            quat = LQuaternionf(qw, qx, qy, qz)
            vec3 = quat.getHpr()
            pitch = vec3[1]
            roll = vec3[2] 
            yaw = vec3[0]
            
            writer.writerow([time, roll, pitch, yaw])