import serial
import csv
import traceback

port = "/dev/cu.usbmodem101" 
baudrate = 921600

csv_filename = "files.csv"  
with open(csv_filename, "r") as file:
    first_line = file.readline().strip() 

file_paths = first_line.split(",")

# This corresponds to approximately 52 seconds of data
N = 19500

try:
    with serial.Serial(port, baudrate, timeout=2) as ser, open(file_paths[4], "w", newline="") as data_file, open(file_paths[1], "w", newline="") as f_mag, open(file_paths[0], "w", newline="") as f_acc, open(file_paths[2], "w", newline="") as f_gyr:
        writer_data = csv.writer(data_file)
        writer_data.writerow(["Time", "AccX", "AccY","AccZ","GyrX", "GyrY", "GyrZ", "MagX","MagY", "MagZ"])  # Cabecera

        writer_mag = csv.writer(f_mag)
        writer_acc = csv.writer(f_acc)
        writer_gyr = csv.writer(f_gyr)

        i = 0
        while i < N:
            try:
                line = ser.readline().decode().strip()
                if line:
                    data = line.split(",")
                    if len(data) != 10:
                        continue
                    if i == 0:
                        tiempo_inicial = float(data[0])
                        data[0] = "0.0"
                    else:
                        data[0] = str(float(data[0]) - float(tiempo_inicial))

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

