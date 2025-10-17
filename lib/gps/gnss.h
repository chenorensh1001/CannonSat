#pragma once

namespace gnss {

/**
 * @brief GPS location data structure
 */
struct Location {
    double latitude  = 0.0;   /* Latitude in degrees */
    double longitude = 0.0;   /* Longitude in degrees */
    double altitude  = 0.0;   /* Altitude in meters */
    double speed     = 0.0;   /* Speed in meters per second */
    double course    = 0.0;   /* Course over ground in degrees */
    bool valid       = false; /* True if location fix is valid */
};

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
