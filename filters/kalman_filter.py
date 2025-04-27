# Kalman Filter Implementation 

import numpy as np
from time import sleep, time
from math import sin, cos, tan, pi
import csv
import math

csv_filename = "files.csv"  
with open(csv_filename, "r") as file:
    first_line = file.readline().strip()

file_paths = first_line.split(",")

input_file = file_paths[4]
output_file = file_paths[10]

# Initialise matrices and variables
C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
P = np.eye(4)
Q = np.eye(4)
R = np.eye(2)

state_estimate = np.array([[0], [0], [0], [0]])

phi_hat = 0.0
theta_hat = 0.0

# Measured sampling time
dt = 0.0
last_time = 0

with open(output_file, mode='w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(["Time", "Phi (degrees)", "Theta (degrees)"])

    with open(input_file, mode='r', newline='') as f:
        reader = csv.reader(f)
        next(reader)

        for i, row in enumerate(reader):
            time = float(row[0])
            AccX = float(row[1])
            AccY = float(row[2])
            AccZ = float(row[3])
            p = float(row[4]) / (180.0 * 131.0)
            q = float(row[5]) / (180.0 * 131.0)
            r = float(row[6]) / (180.0 * 131.0)

            phi_acc = math.atan2(AccY, math.sqrt(pow(AccX,2.0) + pow(AccZ, 2.0)))
            theta_acc = math.atan2(-AccX, math.sqrt(pow(AccY,2.0) + pow(AccZ, 2.0)))

            # Sampling time
            dt = time - last_time
            last_time = time

            # calculate Euler angle derivatives
            phi_dot = (p + math.sin(phi_hat) * math.tan(theta_hat) * q + math.cos(phi_hat) * math.tan(theta_hat) * r) # % (2 * pi)
            theta_dot = (math.cos(phi_hat) * q - math.sin(phi_hat) * r) # % (2 * pi)

            # Kalman filter
            A = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
            B = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])

            gyro_input = np.array([[phi_dot], [theta_dot]])
            state_estimate = A.dot(state_estimate) + B.dot(gyro_input)
            P = A.dot(P.dot(np.transpose(A))) + Q

            measurement = np.array([[phi_acc], [theta_acc]])
            y_tilde = measurement - C.dot(state_estimate)
            S = R + C.dot(P.dot(np.transpose(C)))
            K = P.dot(np.transpose(C).dot(np.linalg.inv(S)))
            state_estimate = state_estimate + K.dot(y_tilde)
            P = (np.eye(4) - K.dot(C)).dot(P)

            phi_hat = state_estimate[0]
            theta_hat = state_estimate[2]

            writer.writerow([time, str(round(phi_hat[0] * 180.0 / pi, 3)), str(round(theta_hat[0] * 180.0 / pi, 3))])