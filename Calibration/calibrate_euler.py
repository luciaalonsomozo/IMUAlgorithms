import csv
import math

phi_offset = 0.0
theta_offset = 0.0
N = 3000

with open("data/still/data.csv", mode='r', newline='') as f:
    lector = csv.reader(f)
    next(lector)
    for i, fila in zip(range(N), lector): 
            tiempo = float(fila[0])
            last_time = tiempo
            ax = float(fila[1])
            ay = float(fila[2])
            az = float(fila[3])

    [phi_acc, theta_acc] = [math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0)), math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))]
    phi_offset += phi_acc
    theta_offset += theta_acc

phi_offset = float(phi_offset) / float(N)
theta_offset = float(theta_offset) / float(N)

print(phi_offset, theta_offset)