#include <bitset>
#include <Arduino.h>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>

#include "main.h"

bool readWestHandshake() {
  setRow(5);
  delayMicroseconds(3);
  return readCols()[3];
}

bool readEastHandshake() {
  setRow(6);
  delayMicroseconds(3);
  return readCols()[3];
}

void setWestHandshake(uint8_t pinVal) {
  digitalWrite(REN_PIN, 0);         
  setRow(5);                        // Set row address
  delayMicroseconds(3);             // Wait for column inputs to stabilise
  digitalWrite(OUT_PIN, pinVal);    // Set value to latch in DFF
  digitalWrite(REN_PIN, 1);         // Enable selected row
}

void setEastHandshake(uint8_t pinVal) {
  digitalWrite(REN_PIN, 0);
  setRow(6);                        // Set row address
  delayMicroseconds(3);             // Wait for column inputs to stabilise
  digitalWrite(OUT_PIN, pinVal);    // Set value to latch in DFF
  digitalWrite(REN_PIN, 1);         // Enable selected row
}

void determinePosition() {

  // Set W and E handshake ON
  setWestHandshake(1);
  setEastHandshake(1);

  // Set delay for other handshake outputs
  delay(1);             

  bool westEnabled = readWestHandshake();
  
  if (westEnabled == false) {
    
    bool eastEnabled = readEastHandshake();
    
    uint32_t id = HAL_GetUIDw0();
    if (eastEnabled) {
      // Only module, end handshaking here
      id = 0xFFFF;
      device_count = 0;
    } else {
      // First module in series, mark position 0
      uint8_t TX_Message[8];
      memcpy(TX_Message, &id, sizeof(uint32_t));
      TX_Message[4] = 0;
      CAN_TX(0x122, TX_Message);
    } 

    device_id[device_count] = id;
    device_position[device_count] = 0;

    // Set East handshake output OFF
    setEastHandshake(0);
  
  }
  /*
  else {
    while (readWestHandshake()) {
      uint8_t RX_Message[8];
	    uint32_t ID;
      CAN_RX(ID, RX_Message);
      if (ID == 0x122) {
        if (device_count < MAX_DEVICES) {

          uint32_t received_id;

          // Extract first 4 bytes (little endian)
          received_id  =  (uint32_t)RX_Message[0];
          received_id |= ((uint32_t)RX_Message[1] << 8);
          received_id |= ((uint32_t)RX_Message[2] << 16);
          received_id |= ((uint32_t)RX_Message[3] << 24);

          device_id[device_count] = received_id;
          device_position[device_count]  = RX_Message[4];

          device_count++;
        }
      }
    }
    
    // Broadcast new handshaking message
    uint8_t TX_Message[8];
    uint32_t id = HAL_GetUIDw0();
    memcpy(TX_Message, &id, sizeof(uint32_t));
    TX_Message[4] = device_count;
    CAN_TX(0x122, TX_Message);

    device_id[device_count] = id;
    device_position[device_count] = 1;

    // Set East Handshake off
    setEastHandshake(0);

    if (!readEastHandshake()) {
      
      // 7 bytes + null terminator
      uint8_t TX_Message[8] = "CAN-FIN";
      CAN_TX(0x122, TX_Message);
    }
  }
    */
}
