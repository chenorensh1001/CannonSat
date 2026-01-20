#ifndef CONFIG_H
#define CONFIG_H
#pragma once
#define TEAM_ID 4


//TEAM ID//

/* --- GNSS Configuration --- */
#define GNSS_BAUD_RATE 9600
#define PM_SENSOR_BAUD_RATE 9600


/* --- IMU Configuration --- */
#define IMU_I2C_ADDRESS 0x68

/* --- LoRa Configuration --- */
#define RFM95_CS 10
#define RFM95_INT 25
#define RFM95_RST 24
#define LORA_FREQ 868300000 // Hz
#define LORA_TX_POWER 14 // dBm
#define LORA_SPREADING_FACTOR 8 // 7-12
#define LORA_CODING_RATE 6 // 4/6
#define LORA_BANDWIDTH 250E3 // Hz
#endif


//   Serial.println("LoRa initialized");
//         LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
//         LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
//         LoRa.setSignalBandwidth(LORA_BANDWIDTH);
//         LoRa.setCodingRate4(LORA_CODING_RATE);
//         LoRa.endPacket(false); 