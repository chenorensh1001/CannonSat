#pragma once
#include <TinyGPS++.h>


namespace gnss {

/**
 * @brief GPS location data structure
 */
struct Location {
    double latitude;
    double longitude;
    double altitude;          // meters
    double speed;             // m/s (horizontal)
    double course;            // degrees
    bool   valid;
    uint32_t timestamp;       // Unix seconds (with fallback)
    float verticalVelocity;   // m/s (GNSS or IMU fallback)
};

uint32_t convertToUnix(TinyGPSDate& d, TinyGPSTime& t);


/**
 * @brief Initialize the GNSS module
 * 
 * Sets up the serial connection to the GPS.
 * 
 * @return 0 on success, non-zero on failure
 */
int setup();

/**
 * @brief Poll the GNSS module and return the latest valid location
 * 
 * This function reads all available bytes from the GPS serial port and
 * parses them using TinyGPS++. If a new valid fix is available, it returns
 * a Location struct with valid = true. Otherwise, valid = false.
 * 
 * @return Location struct with current valid data, or valid = false if no new fix
 */
Location read();

/**
 * @brief End communication with the GNSS module
 * 
 * Closes the serial connection.
 */
void end();

}
