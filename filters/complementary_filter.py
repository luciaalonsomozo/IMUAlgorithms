import csv
import math

csv_filename = "carpetas_datos.csv"  
with open(csv_filename, "r") as file:
    first_line = file.readline().strip()  
    
file_paths = first_line.split(",")
input_file = file_paths[4]
output_file = file_paths[5]

# Complementary filter alpha
alpha = 0.5

thetaHat_rad = 0.0
phiHat_rad = 0.0
last_time = 0.0
N = 3000

GyrX_offset = -11.03 / 1000.0 
GyrY_offset = -0.6533333333333333 / 1000.0
GyrZ_offset = -7.637 / 1000.0

with open(output_file, mode='w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(["Time", "Phi (degrees)", "Theta (degrees)"])

    with open(input_file, mode='r', newline='') as f:
        reader = csv.reader(f)
        next(reader)

        for i,row in enumerate(reader):
            time = float(row[0])
            AccX = float(row[1])/1000.0
            AccY = float(row[2])/1000.0
            AccZ = float(row[3])/1000.0
            GyrX = float(row[4])/1000.0
            GyrY = float(row[5])/1000.0
            GyrZ = float(row[6])/1000.0
            MagX = float(row[7])/1000.0
            MagY = float(row[8])/1000.0
            MagZ = float(row[9])/1000.0

            phi_acc_rad = math.atan2(AccY, math.sqrt(pow(AccX,2.0) + pow(AccZ, 2.0)))
            theta_acc_rad = math.atan2(-AccX, math.sqrt(pow(AccY,2.0) + pow(AccZ, 2.0)))

            GyrX -=  GyrX_offset
            GyrY -=  GyrY_offset
            GyrZ -=  GyrZ_offset

            phiDot_rps = GyrX + math.tan(thetaHat_rad)*(math.sin(phiHat_rad)*GyrY + math.cos(phiHat_rad)*GyrZ)
            thetaDot_rps = math.cos(phiHat_rad)*GyrY - math.sin(phiHat_rad)*GyrZ

            dt = time - last_time
            phiHat_rad = alpha*phi_acc_rad + (1.0 - alpha)*(phiHat_rad + dt*phiDot_rps)
            thetaHat_rad = alpha*theta_acc_rad + (1.0 - alpha)*(thetaHat_rad + dt*thetaDot_rps)
            
            last_time = time
            
            phiHat_rad = ((phiHat_rad + math.pi) % (2.0*math.pi)) - math.pi
            thetaHat_rad = ((thetaHat_rad + math.pi) % (2.0*math.pi)) - math.pi
            
            writer.writerow([time, math.degrees(phiHat_rad), math.degrees(thetaHat_rad)])