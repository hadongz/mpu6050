#include <stdio.h>
#include <bcm2835.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

int main(int argc, char **argv) {
    printf("MPU6050 3-axis acceleromter example program\n");
    I2Cdev::initialize();
    MPU6050 mpu;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (mpu.testConnection()) {
        printf("MPU6050 connection test succesful\n");
    } else {
        fprintf(stderr, "MPU6050  connection test failed!\n");
    }

    mpu.initialize();

    while (true) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Get the current full scale settings to calculate conversion factors
        uint8_t accelRange = mpu.getFullScaleAccelRange();
        uint8_t gyroRange = mpu.getFullScaleGyroRange();
        
        // Determine accelerometer conversion factor
        float accelScale;
        switch (accelRange) {
            case MPU6050_ACCEL_FS_2: accelScale = 16384.0; break;  // ±2g
            case MPU6050_ACCEL_FS_4: accelScale = 8192.0; break;   // ±4g
            case MPU6050_ACCEL_FS_8: accelScale = 4096.0; break;   // ±8g
            case MPU6050_ACCEL_FS_16: accelScale = 2048.0; break;  // ±16g
            default: accelScale = 16384.0; // Default to ±2g
        }
        
        // Determine gyroscope conversion factor
        float gyroScale;
        switch (gyroRange) {
            case MPU6050_GYRO_FS_250: gyroScale = 131.0; break;    // ±250°/s
            case MPU6050_GYRO_FS_500: gyroScale = 65.5; break;     // ±500°/s
            case MPU6050_GYRO_FS_1000: gyroScale = 32.8; break;    // ±1000°/s
            case MPU6050_GYRO_FS_2000: gyroScale = 16.4; break;    // ±2000°/s
            default: gyroScale = 131.0; // Default to ±250°/s
        }
        
        // Convert raw values to physical units
        float accel_x = ax / accelScale;
        float accel_y = ay / accelScale;
        float accel_z = az / accelScale;
        
        float gyro_x = gx / gyroScale;
        float gyro_y = gy / gyroScale;
        float gyro_z = gz / gyroScale;
        
        // Calculate orientation angles from accelerometer
        float roll = atan2(accel_y, accel_z) * 180.0 / M_PI;
        float pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0 / M_PI;
        
        // Print all values
        printf("Accelerometer: X=%.2fg, Y=%.2fg, Z=%.2fg\n", accel_x, accel_y, accel_z);
        printf("Gyroscope: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s\n", gyro_x, gyro_y, gyro_z);
        printf("Roll: %.2f°, Pitch: %.2f°\n", roll, pitch);
        printf("--------------------------------------------------\n");

        fflush(stdout);
        bcm2835_delay(100);
    }

    return 1;
}
