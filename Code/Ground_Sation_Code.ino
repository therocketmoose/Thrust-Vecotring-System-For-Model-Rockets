
#include <RadioLib.h> // Install "RadioLib" library
#include <SPI.h>

// --- PIN DEFINITIONS (ESP32 Standard VSPI) ---
// Change these if using Arduino Uno/Nano
#define LORA_CS   5
#define LORA_DIO0 2
#define LORA_RST  14
#define LORA_DIO1 4  // Optional, often not needed for basic receive

// Instantiate the radio module
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for laptop connection

  Serial.println(F("Initializing Ground Station..."));

  // --- INITIALIZE LORA ---
  // Must match the settings in your Teensy Flight Computer exactly!
  // Frequency: 915.0 (US), 868.0 (EU), 433.0 (Asia/Other)
  int state = radio.begin(915.0); 

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa Init Success!"));
    
    // Optional: Set spreading factor/bandwidth if you changed them on the rocket
    // radio.setSpreadingFactor(7);
    // radio.setBandwidth(125.0);
  } else {
    Serial.print(F("LoRa Init Failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  // Check if a packet has been received
  String receivedData;
  int state = radio.receive(receivedData);

  if (state == RADIOLIB_ERR_NONE) {
    // --- PACKET RECEIVED SUCCESSFULLY ---
    
    // 1. Print the raw data (CSV format from rocket)
    // Format: Time,Pitch,Yaw,Altitude,State
    Serial.print("DATA: ");
    Serial.print(receivedData);

    // 2. Print Signal Strength (RSSI) and Signal-to-Noise (SNR)
    // RSSI: Closer to 0 is better (e.g., -40 is strong, -110 is weak)
    Serial.print(" | RSSI: ");
    Serial.print(radio.getRSSI());
    Serial.print(" dBm");
    
    Serial.print(" | SNR: ");
    Serial.print(radio.getSNR());
    Serial.println(" dB");

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // No packet received in this loop, just keep listening
    // This is normal, do nothing.
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println(F("Error: CRC Mismatch (Corrupted Packet)"));
  } else {
    // Some other error
    Serial.print(F("Error: "));
    Serial.println(state);
  }
}