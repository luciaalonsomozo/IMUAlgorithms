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

    // Define calibration
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    double scale_gyr = 1.0 / 16.4;
    const FusionVector gyroscopeOffset = {-11.03 * scale_gyr, -0.6533333333333333 * scale_gyr, -7.637 * scale_gyr};
    
    const FusionVector hardIronOffset = {12.55537129 , -68.81162222 , -57.65193058};
    const FusionMatrix softIronMatrix = {1.08366655, -0.0187489,  0.01229909,-0.0187489 ,  1.04376805 ,-0.00784824, 0.01229909, -0.00784824, 1.05946304};
    
    double scale_acc = 1.0 / 8192.0;
    const FusionVector accelerometerOffset = {187.39 * scale_acc, -224.86 * scale_acc, 113.28 * scale_acc};
    const FusionMatrix accelerometerMisalignment = {1.02497 , -0.00726 , 0.01056 ,-0.00726 , 1.0165 , -0.00169, 0.01056 , -0.00169 , 0.94969 };

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


    FILE *file_datos = fopen("files.csv", "r");
    if (!file_datos) {
        perror("Error opening the file.\n");
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

    // Open CSV file for writing
    FILE *output_file = fopen(file_paths[7], "w");
    if (output_file == NULL)
    {
        perror("Error opening the output file.\n");
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

    double last_time = 0;

    double Qw, Qx, Qy, Qz;

    fgets(line, sizeof(line), input_file);
   
    // Leer línea por línea
    while (fgets(line, sizeof(line), input_file) != NULL)
    {   
        
        int valores_leidos = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &time, &AccX, &AccY, &AccZ, &GyrX, &GyrY, &GyrZ, &MagX, &MagY, &MagZ);

        double delta_time = (time - last_time); // Calculamos el delta en segundo
        last_time = time;

        FusionVector gyroscope = {GyrX * scale_gyr, GyrY * scale_gyr, GyrZ * scale_gyr};    
        FusionVector accelerometer = {AccX * scale_acc, AccY * scale_acc, AccZ* scale_acc}; 
        FusionVector magnetometer = {MagX, MagY, MagZ}; 

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
       
        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Write to CSV
        fprintf(output_file, "%.5f,%.5f,%.5f,%.5f\n", time, euler.angle.roll, euler.angle.pitch, -euler.angle.yaw);
    }

    fclose(input_file); // Cerrar el archivo
    fclose(output_file);
}
