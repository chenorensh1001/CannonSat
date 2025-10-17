#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

#include "imu.h"
#include "config.h" 

namespace imu {
    Adafruit_ICM20948 icm;

    int setup() {
        if(!icm.begin_I2C()) {
            Serial.println("Failed to find ICM20948");
            return 1; 
        }
        Serial.println("ICM20948 Found!");

        Serial.print("Accelerometer range set to: ");
        switch (icm.getAccelRange()) {
            case ICM20948_ACCEL_RANGE_2_G:
                Serial.println("+-2G");
                break;
            case ICM20948_ACCEL_RANGE_4_G:
                Serial.println("+-4G");
                break;
            case ICM20948_ACCEL_RANGE_8_G:
                Serial.println("+-8G");
                break;
            case ICM20948_ACCEL_RANGE_16_G:
                Serial.println("+-16G");
                break;
        }
        Serial.println("accel OK");

        uint16_t accel_divisor = icm.getAccelRateDivisor();
        float accel_rate = 1125 / (1.0 + accel_divisor);
        Serial.print("Accelerometer data rate divisor set to: ");
        Serial.println(accel_divisor);
        Serial.print("Accelerometer data rate (Hz) is approximately: ");
        Serial.println(accel_rate);

        Serial.print("Gyro range set to: ");
        switch (icm.getGyroRange()) {
            case ICM20948_GYRO_RANGE_250_DPS:
                Serial.println("250 degrees/s");
                break;
            case ICM20948_GYRO_RANGE_500_DPS:
                Serial.println("500 degrees/s");
                break;
            case ICM20948_GYRO_RANGE_1000_DPS:
                Serial.println("1000 degrees/s");
                break;
            case ICM20948_GYRO_RANGE_2000_DPS:
                Serial.println("2000 degrees/s");
                break;
        }
        Serial.println("gyro OK");

        uint8_t gyro_divisor = icm.getGyroRateDivisor();
        float gyro_rate = 1100 / (1.0 + gyro_divisor);
        Serial.print("Gyro data rate divisor set to: ");
        Serial.println(gyro_divisor);
        Serial.print("Gyro data rate (Hz) is approximately: ");
        Serial.println(gyro_rate);

        Serial.print("Magnetometer data rate set to: ");
        switch (icm.getMagDataRate()) {
            case AK09916_MAG_DATARATE_SHUTDOWN:
            Serial.println("Shutdown");
            break;
            case AK09916_MAG_DATARATE_SINGLE:
            Serial.println("Single/One shot");
            break;
            case AK09916_MAG_DATARATE_10_HZ:
            Serial.println("10 Hz");
            break;
            case AK09916_MAG_DATARATE_20_HZ:
            Serial.println("20 Hz");
            break;
            case AK09916_MAG_DATARATE_50_HZ:
            Serial.println("50 Hz");
            break;
            case AK09916_MAG_DATARATE_100_HZ:
            Serial.println("100 Hz");
            break;
        }
        Serial.println("mag OK");

        return 0; 
        }

    void read(sensors_event_t &accel,
          sensors_event_t &gyro,
          sensors_event_t &mag,
          sensors_event_t &temp) {
    icm.getEvent(&accel, &gyro, &temp, &mag);
    }   

    void config( icm20948_accel_range_t accel_range,
                icm20948_gyro_range_t gyro_range,
                ak09916_data_rate_t mag_data_rate,
                uint16_t accel_divisor,
                uint8_t gyro_divisor
                ) {
        icm.setAccelRange(accel_range);
        icm.setGyroRange(gyro_range);
        icm.setMagDataRate(mag_data_rate);
        icm.setAccelRateDivisor(accel_divisor);
        icm.setGyroRateDivisor(gyro_divisor);
    }

}