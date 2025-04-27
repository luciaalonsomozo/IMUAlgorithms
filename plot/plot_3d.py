import csv
import matplotlib.pyplot as pyplot
import numpy

csv_filename = "files.csv"  

with open(csv_filename, "r") as file:
    first_line = file.readline().strip() 

file_paths = first_line.split(",")
data_file = file_paths[4]

data = numpy.genfromtxt(data_file, delimiter=",", skip_header=1)

# For magnetometer data[:, 7:10]
start_index = 7
end_index = 10
magnetometer = data[:, start_index:end_index]

fig = pyplot.figure()
ax = fig.add_subplot(111, projection='3d')

x = magnetometer[:, 0]
y = magnetometer[:, 1]
z = magnetometer[:, 2]

ax.plot3D(x, y, z, color='blue')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

pyplot.show()