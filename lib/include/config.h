#ifndef CONFIG_H
#define CONFIG_H

/* --- GNSS Configuration --- */
#define GNSS_BAUD_RATE 9600

/* --- IMU Configuration --- */
#define IMU_I2C_ADDRESS 0x68

/* --- LoRa Configuration --- */
#define RFM95_CS 10
#define RFM95_INT 14
#define RFM95_RST 15
#define LORA_FREQ 868.0 // MHz
#define LORA_TX_POWER 14 // dBm
#define LORA_SPREADING_FACTOR 11
#define LORA_CODING_RATE 5 // 4/5
#define LORA_BANDWIDTH 125E3 // Hz


#endif