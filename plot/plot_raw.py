import csv
import matplotlib.pyplot as plt

csv_filename = "files.csv"  

with open(csv_filename, "r") as file:
    first_line = file.readline().strip()

file_paths = first_line.split(",")

def plot_raw_csv(filename):
    X = []
    Y = []
    Z = []
    
    with open(filename, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if len(row) < 3:
                continue
            try:
                X.append(float(row[0]))
                Y.append(float(row[1]))
                Z.append(float(row[2]))
            except ValueError:
                continue

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(X, label='X')
    plt.plot(Y, label='Y')
    plt.plot(Z, label='Z')
    plt.xlabel('Sample Index')
    plt.ylabel('Reading')
    plt.title('Raw Data')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # file_paths[1] for raw gyroscope data
    plot_raw_csv(file_paths[1])
    
