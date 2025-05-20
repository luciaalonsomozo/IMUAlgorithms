import csv
import math
import matplotlib.pyplot as plt
import numpy as np

csv_filename = "files.csv"
with open(csv_filename, "r") as file:
    first_line = file.readline().strip()

file_paths = first_line.split(",")
data_file = file_paths[4]
euler_file = file_paths[6] 

alpha_values = [0.02, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.98]

results_phi = {}
results_theta = {}
error_phi = {}
error_theta = {}

euler_time = []
euler_phi = []
euler_theta = []

with open(euler_file, mode='r', newline='') as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        euler_time.append(float(row[0]))
        euler_phi.append(float(row[1]))
        euler_theta.append(float(row[2]))


for alpha in alpha_values:
    thetaHat_rad = 0.0
    phiHat_rad = 0.0
    tiempo_anterior = 0.0
    N = 3000
    GyrX_offset = 0.0
    GyrY_offset = 0.0
    GyrZ_offset = 0.0

    with open(data_file, mode='r', newline='') as f:
        reader = csv.reader(f)
        next(reader)
        for i, row in enumerate(reader):
            if i >= N:
                break
            GyrX_offset += float(row[4]) / 1000.0
            GyrY_offset += float(row[5]) / 1000.0
            GyrZ_offset += float(row[6]) / 1000.0

    GyrX_offset /= N
    GyrY_offset /= N
    GyrZ_offset /= N

    time_list = []
    phi_list = []
    theta_list = []

    with open(data_file, mode='r', newline='') as f:
        reader = csv.reader(f)
        next(reader)

        for row in reader:
            time = float(row[0])
            AccX = float(row[1]) / 1000.0
            AccY = float(row[2]) / 1000.0
            AccZ = float(row[3]) / 1000.0
            GyrX = float(row[4]) / 1000.0 - GyrX_offset
            GyrY = float(row[5]) / 1000.0 - GyrY_offset
            GyrZ = float(row[6]) / 1000.0 - GyrZ_offset

            phi_acc_rad = math.atan2(AccY, math.sqrt(AccX**2 + AccZ**2))
            theta_acc_rad = math.atan2(-AccX, math.sqrt(AccY**2 + AccZ**2))

            dt = (time - tiempo_anterior) / 1000.0
            if dt < 0:
                dt = 0.001

            phiDot_rps = GyrX + math.tan(thetaHat_rad) * (math.sin(phiHat_rad) * GyrY + math.cos(phiHat_rad) * GyrZ)
            thetaDot_rps = math.cos(phiHat_rad) * GyrY - math.sin(phiHat_rad) * GyrZ

            phiHat_rad = alpha * phi_acc_rad + (1.0 - alpha) * (phiHat_rad + dt * phiDot_rps)
            thetaHat_rad = alpha * theta_acc_rad + (1.0 - alpha) * (thetaHat_rad + dt * thetaDot_rps)

            tiempo_anterior = time

            phiHat_rad = ((phiHat_rad + math.pi) % (2.0 * math.pi)) - math.pi
            thetaHat_rad = ((thetaHat_rad + math.pi) % (2.0 * math.pi)) - math.pi

            time_list.append(time)
            phi_list.append(math.degrees(phiHat_rad))
            theta_list.append(math.degrees(thetaHat_rad))

    results_phi[alpha] = (time_list, phi_list)
    results_theta[alpha] = (time_list, theta_list)

    euler_phi_interp = np.interp(time_list, euler_time, euler_phi)
    euler_theta_interp = np.interp(time_list, euler_time, euler_theta)

    error_phi[alpha] = np.mean(np.abs(np.array(phi_list) - np.array(euler_phi_interp)))
    error_theta[alpha] = np.mean(np.abs(np.array(theta_list) - np.array(euler_theta_interp)))

best_alpha_phi = min(error_phi, key=error_phi.get)
best_alpha_theta = min(error_theta, key=error_theta.get)

print(f"The best alpha for Phi is {best_alpha_phi} with error {error_phi[best_alpha_phi]:.2f}")
print(f"The best alpha for Theta is {best_alpha_theta} with error {error_theta[best_alpha_theta]:.2f}")

plt.figure(figsize=(12, 6))
cmap = plt.get_cmap("tab10")

plt.subplot(2, 1, 1)
for i, alpha in enumerate(alpha_values):
    time_list, phi_list = results_phi[alpha]
    plt.plot(time_list, phi_list, label=f"Alpha {alpha}", color=cmap(i))
plt.plot(euler_time, euler_phi, 'k--', label="Euler Direct")
plt.xlabel("Time (s)")
plt.ylabel("Roll (degrees)")
plt.title("Roll comparation with different Alpha values")
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
for i, alpha in enumerate(alpha_values):
    time_list, theta_list = results_theta[alpha]
    plt.plot(time_list, theta_list, label=f"Alpha {alpha}", color=cmap(i))
plt.plot(euler_time, euler_theta, 'k--', label="Euler Direct")
plt.xlabel("Time (s)")
plt.ylabel("Pitch (degrees)")
plt.title("Pitch comparation with different Alpha values")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
