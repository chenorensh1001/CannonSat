#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "bmp.h"
#include "gnss.h"
#include "imu.h"
#include "lora.h"
#include "mic.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000); // Wait for Serial to initialize
  
  lora::setup();
  Serial.println("Setup complete.");
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Sending LoRa message...");
  const char* msg = "Hello, LoRa!";
  size_t len = strlen(msg);
  if (lora::send(msg, len)) {
      Serial.println("Message sent successfully.");
  } else {
      Serial.println("Failed to send message.");
  }
  
  delay(5000);
}
