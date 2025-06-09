#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// define macros
#define MAX_BUFFER_SIZE 1000


int main(void) {


    //FILE* infile = fopen("flight_data_raw.txt", "r");
    FILE* infile = fopen("flight.txt", "r");
    FILE* outfile = fopen("flight_data.csv", "w+");

  
    char title[MAX_BUFFER_SIZE];
    double 
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z,
        mag_x, mag_y, mag_z,
        temp, flight_phase; 

    int time_s;


    if (infile == NULL) {
        printf("Path to file corrupted");
        return -1;
    }

    fgets(title, sizeof(title), infile);

    int count = 0;
    
    while(fscanf(infile, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%*c", &time_s, 
        &accel_x, &accel_y, &accel_z,
        &gyro_x, &gyro_y, &gyro_z,
        &mag_x, &mag_y, &mag_z,
        &temp, &flight_phase) == 12) 
    {
        //data corruption after 1580, so its not proepr data storage
        // need elp 
        
        fprintf(outfile, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", time_s, 
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            mag_x, mag_y, mag_z,
            temp, flight_phase);

        
        printf("%d\n", time_s);
        
        printf("%d\n", count);
        count++;
    }

    printf("%d\n", time_s);

    printf("%d", count);
    return 0;
}