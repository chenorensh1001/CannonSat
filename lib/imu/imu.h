#pragma once
#include <Adafruit_ICM20948.h>

namespace imu {

/**
 * @brief Initialize the ICM20948 IMU sensor over I2C
 * 
 * Sets up communication with the IMU and verifies that the device is connected.
 * Prints sensor ranges and data rates to Serial for debugging.
 * 
 * @return 0 on success, non-zero on failure
 */
int setup();

/**
 * @brief Read a single set of sensor measurements
 * 
 * This function fills the provided references with the latest accelerometer,
 * gyroscope, magnetometer, and temperature measurements.
 * 
 * @param accel Reference to a sensors_event_t struct to store accelerometer data
 * @param gyro  Reference to a sensors_event_t struct to store gyroscope data
 * @param mag   Reference to a sensors_event_t struct to store magnetometer data
 * @param temp  Reference to a sensors_event_t struct to store temperature data
 */
void read(sensors_event_t &accel,
          sensors_event_t &gyro,
          sensors_event_t &mag,
          sensors_event_t &temp);

/**
 * @brief Configure IMU sensor settings
 * 
 * Allows customizing the accelerometer range, gyroscope range, magnetometer data rate,
 * and the accelerometer/gyroscope output rate divisors.
 * 
 * @param accel_range  Accelerometer range (ICM20948 enum type)
 * @param gyro_range   Gyroscope range (ICM20948 enum type)
 * @param mag_data_rate Magnetometer output data rate (AK09916 enum type)
 * @param accel_divisor Divisor for accelerometer output rate (ODR = 1125 / (1+divisor))
 * @param gyro_divisor  Divisor for gyroscope output rate (ODR = 1100 / (1+divisor))
 */
void config(icm20948_accel_range_t accel_range,
            icm20948_gyro_range_t gyro_range,
            ak09916_data_rate_t mag_data_rate,
            uint16_t accel_divisor,
            uint8_t gyro_divisor);

}
