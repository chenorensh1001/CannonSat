#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "bmp.h"
#include "gnss.h"
#include "imu.h"
#include "lora_driver.h"
#include "mic.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000); // Wait for Serial to initialize
  
  // Add your setup code here

  Serial.println("Setup complete.");
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Looping...");
  
  delay(5000);
}
