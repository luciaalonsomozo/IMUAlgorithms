#include "../../Fusion/Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define SAMPLE_RATE (69) 
#define MAX_LINE 1024 
#define MAX_PATHS 10 
#define MAX_LENGTH 256

int main()
{

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f, 
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);


    FILE *data_file = fopen("carpetas_datos.csv", "r");
    if (!data_file) {
        perror("Error opening the file.\n");
        return 1;
    }

    char data[MAX_LENGTH * MAX_PATHS]; 
    char *file_paths[MAX_PATHS]; 
    int count = 0;

    if (fgets(data, sizeof(data), data_file)) { 
        char *token = strtok(data, ",\n");
        while (token != NULL && count < MAX_PATHS) {
            file_paths[count] = strdup(token); 
            count++;
            token = strtok(NULL, ",\n"); 
        }
    }

    fclose(data_file);

    FILE *output_file = fopen(file_paths[8], "w");
    if (output_file == NULL)
    {
        perror("Error opening the ouput file.\n");
        return 1;
    }

    // Write CSV header
    fprintf(output_file, "Time,Phi (degrees),Theta (degrees),Yaw (degrees)\n");

    char line[MAX_LINE];
    char line2[MAX_LINE];

    double time, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ;
    FILE* input_file = fopen(file_paths[4], "r");
    if (input_file == NULL)
    {
        printf("Error opening the input file.\n");
        return 1;
    }

    if (fgets(line, sizeof(line), input_file) == NULL)
    {
        printf("Error reading the header.\n");
        fclose(input_file);
        return 1;
    }

    double scale_gyr = 1.0 / 16.4;
    double scale_acc = 1.0 / 8192.0;
  
    double last_time = 0;

    fgets(line, sizeof(line), input_file);

    while (fgets(line, sizeof(line), input_file) != NULL)
    {   

        int data_read = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &time, &AccX, &AccY, &AccZ, &GyrX, &GyrY, &GyrZ, &MagX, &MagY, &MagZ);

        double delta_time = (time - last_time); 
        last_time = time;


        FusionVector gyroscope = {GyrX * scale_gyr, GyrY * scale_gyr, GyrZ * scale_gyr};     
        FusionVector accelerometer = {AccX * scale_acc, AccY * scale_acc, AccZ* scale_acc};
        FusionVector magnetometer = {MagX, MagY, MagZ};

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Write to CSV
        fprintf(output_file, "%.5f,%.5f,%.5f,%.5f\n", time, euler.angle.roll, euler.angle.pitch, -euler.angle.yaw);
    }

    fclose(input_file);
    fclose(output_file);
}
