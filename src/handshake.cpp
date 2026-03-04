#include <bitset>
#include <Arduino.h>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>

#include "main.h"

uint8_t device_num = 0;

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

void handshakeTask(void * pvParameters) {

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    // Set W and E handshake ON
    setWestHandshake(1);
    setEastHandshake(1);

    bool westEnabled = readWestHandshake();
    
    if (westEnabled) {
      
      bool eastEnabled = readEastHandshake();
      if (eastEnabled) {
        // Only module detected
      } else {
        // First module detected
      } 

      device_num = 0;

    } else {
      bool eastEnabled = readEastHandshake();
      if (eastEnabled == false) {
        // Last module detected
        device_num = 2;
      } else {
        // Middle module detected
        device_num = 1;
      }
    }
  }
}
