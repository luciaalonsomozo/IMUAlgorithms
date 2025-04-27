#include "../../Fusion/Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define MAX_LINE 1024  
#define MAX_PATHS 10
#define MAX_LENGTH 256

int main() {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);


    FILE *file_datos = fopen("files.csv", "r");
    if (!file_datos) {
        perror("Error opening the file carpetas_datos.csv.");
        return 1;
    }

    char data[MAX_LENGTH * MAX_PATHS]; 
    char *file_paths[MAX_PATHS];
    int count = 0;

    if (fgets(data, sizeof(data), file_datos)) {
        char *token = strtok(data, ",\n");
        while (token != NULL && count < MAX_PATHS) {
            file_paths[count] = strdup(token);
            count++;
            token = strtok(NULL, ",\n");
        }
    }

    fclose(file_datos);

    FILE *inuput_file = fopen(file_paths[4], "r");
    if (inuput_file == NULL) {
        printf("Error opening the input file.\n");
        return 1;
    }

    FILE *output_file = fopen(file_paths[9], "w");
    if (output_file == NULL) {
        perror("Error opening the ouput file.\n");
        return 1;
    }

    // Write CSV header
    fprintf(output_file, "Time,Phi (degrees),Theta (degrees)\n");

    char line[MAX_LINE];
    
    if (fgets(line, sizeof(line), inuput_file) == NULL) {
        printf("Error reading the header.\n");
        fclose(inuput_file);
        return 1;
    }

    double time, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ;

    double last_time = 0;

    while (fgets(line, sizeof(line), inuput_file) != NULL) {
        int data_read = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                    &time, &AccX, &AccY, &AccZ, &GyrX, &GyrY, &GyrZ, &MagX, &MagY, &MagZ);
        
        if(data_read != 10){
            printf("10 values couldn't be read.\n");
            return 1;
        }

        double delta_time = time - last_time;
        last_time = time;

        double scale_gyr = 1.0 / 16.4;
        double scale_acc = 1.0 / 8192.0;

        const FusionVector gyroscope = {GyrX * scale_gyr, GyrY * scale_gyr, GyrZ* scale_gyr};
        const FusionVector accelerometer = {AccX * scale_acc, AccY * scale_acc, AccZ * scale_acc};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Write to CSV
        fprintf(output_file, "%.5f,%.5f,%.5f\n", time, euler.angle.roll, euler.angle.pitch);
    }

    fclose(inuput_file);
    fclose(output_file);

    return 0;
}
