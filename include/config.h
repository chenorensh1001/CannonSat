#ifndef CONFIG_H
#define CONFIG_H

/* --- GNSS Configuration --- */
#define GNSS_BAUD_RATE 9600


/* --- LoRa Configuration --- */
#define RFM69_CS 4
#define RFM69_INT 3
#define LORA_FREQ 868.0 // MHz
#define LORA_TX_POWER 14 // dBm
#define LORA_SPREADING_FACTOR 11
#define LORA_CODING_RATE 5 // 4/5
#define LORA_BANDWIDTH 125E3 // Hz


#endif