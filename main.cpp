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
    bool first_time = true;

    float complementaryRoll = 0;
    float complementaryPitch = 0;
    unsigned long previousTime = 0;

    while (true) {

        unsigned long currentTime = bcm2835_st_read() / 1000;
        float dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        if (dt > 0.2) {
            dt = 0.01;
        }

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

        float alpha = 0.98;

        if (complementaryRoll == 0 && complementaryPitch == 0) {
            complementaryRoll = roll;
            complementaryPitch = pitch;
        } else {
            complementaryRoll = alpha * (complementaryRoll + gyro_x * dt) + (1 - alpha) * roll;
            complementaryPitch = alpha * (complementaryPitch + gyro_y * dt) + (1 - alpha) * pitch;
        }
        
        // First time through, just print the values
        if (first_time) {
            printf("Accelerometer (g):  X: %+6.3f  Y: %+6.3f  Z: %+6.3f\n", accel_x, accel_y, accel_z);
            printf("Gyroscope (°/s):    X: %+7.2f  Y: %+7.2f  Z: %+7.2f\n", gyro_x, gyro_y, gyro_z);
            printf("Orientation (°):    Roll: %+6.2f  Pitch: %+6.2f\n", roll, pitch);
            printf("Complementary Orientation (°):    Complementary Roll: %+6.2f  Complementary Pitch: %+6.2f\n", complementaryRoll, complementaryPitch);
            first_time = false;
        } else {
            // Move up 3 lines
            printf("\033[4A");
            // Print new values (overwriting old ones)
            printf("Accelerometer (g):  X: %+6.3f  Y: %+6.3f  Z: %+6.3f\n", accel_x, accel_y, accel_z);
            printf("Gyroscope (°/s):    X: %+7.2f  Y: %+7.2f  Z: %+7.2f\n", gyro_x, gyro_y, gyro_z);
            printf("Orientation (°):    Roll: %+6.2f  Pitch: %+6.2f\n", roll, pitch);
            printf("Complementary Orientation (°):    Complementary Roll: %+6.2f  Complementary Pitch: %+6.2f\n", complementaryRoll, complementaryPitch);
        }
        
        // Make sure output is displayed immediately
        fflush(stdout);

        // bcm2835_delay(100);
    }

    return 1;
}
