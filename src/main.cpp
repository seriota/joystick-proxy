#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>

// Pin definitions for LoRa module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BAND 915E6

#define DT_RX 16
#define DT_TX 17

HardwareSerial dataSerial(1); // Serial1 on ESP32
byte calculate_crc(uint8_t *data, uint16_t len);
bool validate(char *data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dataSerial.begin(115200, SERIAL_8N1, DT_RX, DT_TX); // RX, TX pins for Serial1n
  SPI.begin(SCK, MISO, MOSI, SS); // Initialize SPI with custom pins
  LoRa.setPins(SS, RST, DIO0); // Set LoRa module pins
  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (1){}
  }
  Serial.println("LoRa init succeeded.");
}

void loop() {
  if(!LoRa.parsePacket()) {
    return; // No packet received
  }

  Serial.println("Packet received.");
  if(!LoRa.available()) {
    return; // No data available
  }

  String data = LoRa.readStringUntil('\n'); // Read the packet until newline
  Serial.print("Received data: ");
  Serial.println(data); // Print the received data

  if (!validate((char *)data.c_str())) {
    Serial.println("Invalid data format or CRC mismatch.");
    return; // Invalid data format or CRC mismatch
  }
  Serial.println("Data is valid.");
  // delay(200);
}

byte calculate_crc(uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;           // Initial value
  uint8_t polynomial = 0x07;    // x^8 + x^2 + x + 1

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool validate(char *data) {
  if (data[0] != '?') {
    return false; // Invalid start character
  } 

  char buffer[100];
  strncpy(buffer, data, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';  // ensure null-termination

  // Step 2: Extract the number between '?' and the first ':'
  char* qmark = strchr(buffer, '?');
  char* colon = strchr(buffer, ':');

  int extracted_crc = 0;

  if (qmark && colon && colon > qmark) {
    char numberBuffer[10];
    size_t len = colon - qmark - 1;  // exclude '?' and get length of number

    if (len < sizeof(numberBuffer)) {
      strncpy(numberBuffer, qmark + 1, len);
      numberBuffer[len] = '\0';  // null-terminate
      extracted_crc = atoi(numberBuffer);
    }

    // Step 3: Remove everything up to and including the first ':'
    // shift the string left in-place
    memmove(buffer, colon + 1, strlen(colon + 1) + 1);  // include null terminator
  }

  Serial.print("Extracted value: ");
  Serial.println(extracted_crc);  // should print 98

  Serial.print("Remaining buffer: ");
  Serial.println(buffer);  // should print "1024:2048:0:4095:512:1"

  //calculate crc
  uint8_t crc = calculate_crc((uint8_t *)buffer, strlen(buffer));
  if (crc != extracted_crc) {
    Serial.print("CRC mismatch: calculated ");
    Serial.print(crc);
    Serial.print(", expected ");
    Serial.println(extracted_crc);
    return false; // CRC mismatch
  }
  return true; // Valid packet
}