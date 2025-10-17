#pragma once

namespace bmp {

/**
 * @brief BMP388 sensor reading structure
 */
struct Reading {
    float temperature = 0.0;    /* Temperature in degrees Celsius */
    float pressure    = 0.0;    /* Pressure in Pascals */
    float altitude    = 0.0;    /* Altitude in meters (calculated from pressure) */
    bool valid        = false;  /* True if reading is valid */
};

/**
 * @brief Initialize BMP388 sensor over I2C
 * 
 * Sets oversampling, filter, and data rate.
 * 
 * @return 0 on success, non-zero on failure
 */
int setup();

/**
 * @brief Perform a single reading from BMP388
 * 
 * Reads temperature and pressure, computes altitude.
 * 
 * @return Reading struct with .valid = true if successful
 */
Reading read();

}
