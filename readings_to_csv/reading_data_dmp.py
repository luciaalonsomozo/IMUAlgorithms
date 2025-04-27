import serial
import csv
import traceback

port = "/dev/cu.usbmodem1101"  
baudrate = 115200 

csv_filename = "files.csv"  
with open(csv_filename, "r") as file:
    first_line = file.readline().strip() 

file_paths = first_line.split(",")

# This corresponds to approximately 52 seconds of data
N = 3000

try:
    with serial.Serial(port, baudrate, timeout=2) as ser, open(file_paths[4], "w", newline="") as data_file, open(file_paths[3], "w", newline="") as quat_file, open(file_paths[1], "w", newline="") as f_mag, open(file_paths[0], "w", newline="") as f_acc, open(file_paths[2], "w", newline="") as f_gyr:
        writer_data = csv.writer(data_file)
        writer_data.writerow(["Time", "AccX", "AccY","AccZ","GyrX", "GyrY", "GyrZ", "MagX","MagY", "MagZ"])  # Cabecera
        
        writer_quat = csv.writer(quat_file)
        writer_quat.writerow(["Time","QuatW", "QuatX", "QuatY", "QuatZ"])  # Cabecera

        writer_mag = csv.writer(f_mag)
        writer_acc = csv.writer(f_acc)
        writer_gyr = csv.writer(f_gyr)

        i = 0
        while i < N: # Reading N samples
            try:
                line = ser.readline().decode().strip()
                if line:
                    data = line.split(",")
                    if len(data) != 14:
                        continue
                    if i == 0:
                        start_time = float(data[0])
                        data[0] = "0.0"
                    else:
                        data[0] = str(float(data[0]) - float(start_time))
                    writer_quat.writerow([data[0]] + data[10:])
                    writer_data.writerow(data[:10])
                    writer_mag.writerow(data[7:10])
                    writer_acc.writerow(data[1:4])
                    writer_gyr.writerow(data[4:7])
                    print(data)  
            except serial.SerialException:
                print("Error with the port.")
                break
            except KeyboardInterrupt:
                print("\nStopped by the user.")
                break
            i = i+1

except serial.SerialException as e:
    print("Port couldn't be opened. Verify the connection.", e)
    traceback.print_exc()
