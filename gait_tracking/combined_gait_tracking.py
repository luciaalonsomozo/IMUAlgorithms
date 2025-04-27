from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
import imufusion
import matplotlib.pyplot as pyplot
import numpy
import csv
import math
from mpl_toolkits.mplot3d import Axes3D

csv_filename = "files.csv"  

with open(csv_filename, "r") as file:
    first_line = file.readline().strip()

file_paths = first_line.split(",")
input_file = file_paths[4]

data = numpy.genfromtxt(input_file, delimiter=",", skip_header=1)

scale_gyr = 1.0 / 16.4
scale_acc = 1.0 / 8192.0

sample_rate = 367 
timestamp = data[:, 0]
gyroscope = data[:,4:7]
accelerometer = data[:, 1:4] * (1 / 1000.0)
magnetometer = data[:, 7:10]

print(len(timestamp))

# Plot sensor data
figure, axes = pyplot.subplots(nrows=3, sharex=True, gridspec_kw={"height_ratios": [6, 6, 6]})

figure.suptitle("Sensors data, Euler angles, and AHRS internal states")

axes[0].plot(timestamp, gyroscope[:, 0], "tab:red", label="Gyroscope X")
axes[0].plot(timestamp, gyroscope[:, 1], "tab:green", label="Gyroscope Y")
axes[0].plot(timestamp, gyroscope[:, 2], "tab:blue", label="Gyroscope Z")
axes[0].set_ylabel("Degrees/s")
axes[0].grid()
axes[0].legend()

axes[1].plot(timestamp, accelerometer[:, 0], "tab:red", label="Accelerometer X")
axes[1].plot(timestamp, accelerometer[:, 1], "tab:green", label="Accelerometer Y")
axes[1].plot(timestamp, accelerometer[:, 2], "tab:blue", label="Accelerometer Z")
axes[1].set_ylabel("g")
axes[1].grid()
axes[1].legend()

axes[2].plot(timestamp, magnetometer[:, 0], "tab:red", label="Magnetometer X")
axes[2].plot(timestamp, magnetometer[:, 1], "tab:green", label="Magnetometer Y")
axes[2].plot(timestamp, magnetometer[:, 2], "tab:blue", label="Magnetometer Z")
axes[2].grid()
axes[2].legend()

def fusion_calibration_inertial(uncalibrated: numpy.ndarray, misalignment: numpy.ndarray, 
                                sensitivity: numpy.ndarray, offset: numpy.ndarray) -> numpy.ndarray:
    adjusted = (uncalibrated - offset) * sensitivity  
    calibrated = numpy.dot(misalignment, adjusted)
    return calibrated

def fusion_calibration_magnetic(uncalibrated: numpy.ndarray, soft_iron_matrix: numpy.ndarray, 
                                hard_iron_offset: numpy.ndarray) -> numpy.ndarray:
    adjusted = uncalibrated - hard_iron_offset
    calibrated = numpy.dot(soft_iron_matrix, adjusted)
    return calibrated

# Instantiate AHRS algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                   0.5,  # gain
                                   2000,  # gyroscope range
                                   10,  # acceleration rejection
                                   100,  # magnetic rejection
                                   5 * sample_rate)  # rejection timeout = 5 seconds

magneticPercentage = 0.01

# Process sensor data
delta_time = numpy.diff(timestamp, prepend=timestamp[0])

euler = numpy.empty((len(timestamp), 3))
internal_states = numpy.empty((len(timestamp), 6))
flags = numpy.empty((len(timestamp), 4))
acceleration = numpy.empty((len(timestamp), 3))

count_magnetometer = 0
count_no_magnetometer = 0

for index in range(len(timestamp)):
     
    gyroscope[index] = fusion_calibration_inertial(gyroscope[index], 
                                                   numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), 
                                                   numpy.array([1, 1, 1]), 
                                                   numpy.array([-11.03 * scale_gyr, 0.6533333333333333 * scale_gyr, -7.637 * scale_gyr]))
    
    magnetometer[index] = fusion_calibration_magnetic(magnetometer[index],
                                                      numpy.array([[ 1.08366655, -0.0187489,  0.01229909],
                                                               [-0.0187489 ,  1.04376805 ,-0.00784824],
                                                                   [ 0.01229909, -0.00784824, 1.05946304]]) ,
                                                    numpy.array([12.56 * 0.15 , -68.81  * 0.15, -57.65 * 0.15]))
    
    accelerometer[index] = fusion_calibration_inertial(accelerometer[index],
                                                      numpy.array([[1.02497 , -0.00726 , 0.01056] ,[-0.00726 , 1.0165 , -0.00169], [0.01056 , -0.00169 , 0.94969]]),
                                                      numpy.array([1, 1, 1]),
                                                      numpy.array([187.39 * scale_acc, -224.86 * scale_acc, 113.28 * scale_acc]))
    
    gyroscope[index] = offset.update(gyroscope[index])

    magnetometer[index][1] = -magnetometer[index][1]  # invert Y axis
    magnetometer[index][2] = -magnetometer[index][2]  # invert Z axis
    
    mod = math.sqrt(pow(magnetometer[index][1],2) + pow(magnetometer[index][2], 2) + pow(magnetometer[index][0], 2))
    
    print(mod)
    
  
    if (mod > (1 - magneticPercentage)*37 and mod < (1 + magneticPercentage)*37):
        ahrs.update(gyroscope[index], accelerometer[index], magnetometer[index], delta_time[index])
        count_magnetometer += 1
    else:
        ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], delta_time[index])
        count_no_magnetometer += 1
        
    euler[index] = ahrs.quaternion.to_euler()

    ahrs_internal_states = ahrs.internal_states
    internal_states[index] = numpy.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_recovery_trigger,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_recovery_trigger,
                                          ])
    ahrs_flags = ahrs.flags
    flags[index] = numpy.array(
        [
            ahrs_flags.initialising,
            ahrs_flags.angular_rate_recovery,
            ahrs_flags.acceleration_recovery,
            ahrs_flags.magnetic_recovery,
        ]
    )

    acceleration[index] = 9.81 * ahrs.earth_acceleration  # convert g to m/s/s

print("Magnetometer: " + str(count_magnetometer))
print("No magnetometer: " + str(count_no_magnetometer))

def plot_bool(axis, x, y, label):
    axis.plot(x, y, "tab:cyan", label=label)
    pyplot.sca(axis)
    pyplot.yticks([0, 1], ["False", "True"])
    axis.grid()
    axis.legend()
    
# Plot Euler angles
figure, axes = pyplot.subplots(nrows=11, sharex=True, gridspec_kw={"height_ratios": [6, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1]})

figure.suptitle("Euler angles, internal states, and flags")

axes[0].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[0].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[0].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[0].set_ylabel("Degrees")
axes[0].grid()
axes[0].legend()

# Plot initialising flag
plot_bool(axes[1], timestamp, flags[:, 0], "Initialising")

# Plot angular rate recovery flag
plot_bool(axes[2], timestamp, flags[:, 1], "Angular rate recovery")

# Plot acceleration rejection internal states and flag
axes[3].plot(timestamp, internal_states[:, 0], "tab:olive", label="Acceleration error")
axes[3].set_ylabel("Degrees")
axes[3].grid()
axes[3].legend()

plot_bool(axes[4], timestamp, internal_states[:, 1], "Accelerometer ignored")

axes[5].plot(timestamp, internal_states[:, 2], "tab:orange", label="Acceleration recovery trigger")
axes[5].grid()
axes[5].legend()

plot_bool(axes[6], timestamp, flags[:, 2], "Acceleration recovery")

# Plot magnetic rejection internal states and flag
axes[7].plot(timestamp, internal_states[:, 3], "tab:olive", label="Magnetic error")
# axes[7].set_ylabel("Degrees")
axes[7].grid()
axes[7].legend()

plot_bool(axes[8], timestamp, internal_states[:, 4], "Magnetometer ignored")

axes[9].plot(timestamp, internal_states[:, 5], "tab:orange", label="Magnetic recovery trigger")
axes[9].grid()
axes[9].legend()

plot_bool(axes[10], timestamp, flags[:, 3], "Magnetic recovery")

# Plot acceleration
_, axes = pyplot.subplots(nrows=4, sharex=True, gridspec_kw={"height_ratios": [6, 1, 6, 6]})

axes[0].plot(timestamp, acceleration[:, 0], "tab:red", label="X")
axes[0].plot(timestamp, acceleration[:, 1], "tab:green", label="Y")
# axes[0].plot(timestamp, acceleration[:, 2], "tab:blue", label="Z")
axes[0].set_title("Acceleration")
axes[0].set_ylabel("m/s/s")
axes[0].grid()
axes[0].legend()

# Identify moving periods
is_moving = numpy.empty(len(timestamp))

for index in range(len(timestamp)):
    is_moving[index] = numpy.sqrt(acceleration[index].dot(acceleration[index])) > 3  # threshold = 3 m/s/s

margin = int(0.1 * sample_rate)  # 100 ms

for index in range(len(timestamp) - margin):
    is_moving[index] = any(is_moving[index:(index + margin)])  # add leading margin

for index in range(len(timestamp) - 1, margin, -1):
    is_moving[index] = any(is_moving[(index - margin):index])  # add trailing margin

# Plot moving periods
axes[1].plot(timestamp, is_moving, "tab:cyan", label="Is moving")
pyplot.sca(axes[1])
pyplot.yticks([0, 1], ["False", "True"])
axes[1].grid()
axes[1].legend()

# Calculate velocity (includes integral drift)
velocity = numpy.zeros((len(timestamp), 3))

for index in range(len(timestamp)):
    if is_moving[index]:  # only integrate if moving
        velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]

# Find start and stop indices of each moving period
is_moving_diff = numpy.diff(is_moving, append=is_moving[-1])


@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1


is_moving_periods = []
is_moving_period = IsMovingPeriod()

for index in range(len(timestamp)):
    if is_moving_period.start_index == -1:
        if is_moving_diff[index] == 1:
            is_moving_period.start_index = index

    elif is_moving_period.stop_index == -1:
        if is_moving_diff[index] == -1:
            is_moving_period.stop_index = index
            is_moving_periods.append(is_moving_period)
            is_moving_period = IsMovingPeriod()

# Remove integral drift from velocity
velocity_drift = numpy.zeros((len(timestamp), 3))

for is_moving_period in is_moving_periods:
    start_index = is_moving_period.start_index
    stop_index = is_moving_period.stop_index

    t = [timestamp[start_index], timestamp[stop_index]]
    x = [velocity[start_index, 0], velocity[stop_index, 0]]
    y = [velocity[start_index, 1], velocity[stop_index, 1]]
    z = [velocity[start_index, 2], velocity[stop_index, 2]]

    t_new = timestamp[start_index:(stop_index + 1)]

    velocity_drift[start_index:(stop_index + 1), 0] = interp1d(t, x)(t_new)
    velocity_drift[start_index:(stop_index + 1), 1] = interp1d(t, y)(t_new)
    velocity_drift[start_index:(stop_index + 1), 2] = interp1d(t, z)(t_new)

velocity = velocity - velocity_drift

# Plot velocity
axes[2].plot(timestamp, velocity[:, 0], "tab:red", label="X")
axes[2].plot(timestamp, velocity[:, 1], "tab:green", label="Y")
# axes[2].plot(timestamp, velocity[:, 2], "tab:blue", label="Z")
axes[2].set_title("Velocity")
axes[2].set_ylabel("m/s")
axes[2].grid()
axes[2].legend()

# Calculate position
position = numpy.zeros((len(timestamp), 3))

for index in range(len(timestamp)):
    position[index] = position[index - 1] + delta_time[index] * velocity[index]

# Plot position
axes[3].plot(timestamp, position[:, 0], "tab:red", label="X")
axes[3].plot(timestamp, position[:, 1], "tab:green", label="Y")
axes[3].plot(timestamp, position[:, 2], "tab:blue", label="Z")
axes[3].set_title("Position")
axes[3].set_xlabel("Seconds")
axes[3].set_ylabel("m")
axes[3].grid()
axes[3].legend()

# Print error as distance between start and final positions
print("Error: " + "{:.3f}".format(numpy.sqrt(position[-1].dot(position[-1]))) + " m")

# Create 3D animation (takes a long time, set to False to skip)
if True:
    figure = pyplot.figure(figsize=(10, 10))

    axes = pyplot.axes(projection="3d")
    axes.set_xlabel("m")
    axes.set_ylabel("m")
    axes.set_zlabel("m")

    x = []
    y = []
    z = []

    scatter = axes.scatter(x, y, z)

    fps = 30
    samples_per_frame = int(sample_rate / fps)

    def update(frame):
        index = frame * samples_per_frame

        axes.set_title("{:.3f}".format(timestamp[index]) + " s")

        x.append(position[index, 0])
        y.append(position[index, 1])
        z.append(position[index, 2])

        scatter._offsets3d = (x, y, z)

        if (min(x) != max(x)) and (min(y) != max(y)) and (min(z) != max(z)):
            axes.set_xlim3d(min(x), max(x))
            axes.set_ylim3d(min(y), max(y))
            axes.set_zlim3d(min(z), max(z))

            axes.set_box_aspect((numpy.ptp(x), numpy.ptp(y), numpy.ptp(z)))

        return scatter

    anim = animation.FuncAnimation(figure, update,
                                   frames=int(len(timestamp) / samples_per_frame),
                                   interval=1000 / fps,
                                   repeat=False)

    anim.save("animation.gif", writer=animation.PillowWriter(fps))

pyplot.show()
