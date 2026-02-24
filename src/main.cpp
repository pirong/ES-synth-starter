#include <atomic>
#include <bitset>
#include <math.h>

#include <Arduino.h>
#include <U8g2lib.h>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

#define NUM_VOICES 8

#define WAVE_SAW      0
#define WAVE_SINE     1
#define WAVE_TRIANGLE 2
#define WAVE_SQUARE   3

#define SINE_FACTOR   2      // Sine is softer than the rest

std::atomic<uint8_t> currentWaveState = WAVE_SAW;

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int KNOB_MODE = 2;
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

HardwareTimer sampleTimer(TIM1);

const float SAMPLE_RATE = 22000.0f;
const float PHASE_RES = 4294967296.0f; // 2^32

const int8_t semitoneOffsets[12] = {
    -9, -8, -7, -6,
    -5, -4, -3, -2, 
    -1, 0, 1, 2
};

const char* semitoneNames[12] = {
  "C", "C#", "D", "D#", 
  "E", "F", "F#", "G",
  "G#", "A", "A#", "B"
};

uint32_t stepSizes[12] = {};
volatile uint32_t currentStepSize[NUM_VOICES];

void initStepSizes(){
    const float SEMITONE_RATIO = 1.059463f; // 2^(1/12)
    for (int i = 0; i < 12; i++){
        float freq = 440.0f * powf(SEMITONE_RATIO, semitoneOffsets[i]);
        stepSizes[i] = (uint32_t)((freq * PHASE_RES) / SAMPLE_RATE);
    }
}

uint8_t sineTable[256];

void initSineTable() {
  for (int i = 0; i < 256; i++) {
    sineTable[i] = 127 * sin(2.0 * PI * i / 256.0) + 128;
  }
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, ((rowIdx)&1));
  digitalWrite(RA1_PIN, ((rowIdx>>1)&1));
  digitalWrite(RA2_PIN, ((rowIdx>>2)&1));
  digitalWrite(REN_PIN, HIGH);
}

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

class Knob {
public:
    Knob(int8_t lower = 0, int8_t upper = 8)
        : minLimit(lower), maxLimit(upper), rotation(0), prevState(0) {
        mutex = xSemaphoreCreateMutex();
    }

    ~Knob() {
        if (mutex) {
            vSemaphoreDelete(mutex);
        }
    }

    void update(uint8_t inputA, uint8_t inputB) {
        uint8_t currState = (inputB << 1) | inputA;
        int8_t change = 0;

        if ((prevState == 0b00 && currState == 0b01) ||
            (prevState == 0b11 && currState == 0b10)) {
            change = 1;
        }
        else if ((prevState == 0b01 && currState == 0b00) ||
                 (prevState == 0b10 && currState == 0b11)) {
            change = -1;
        }

        xSemaphoreTake(mutex, portMAX_DELAY);
        if (change != 0) {
            rotation += change;
            auto value = rotation.load(std::memory_order_relaxed);
            rotation.store(max(minLimit, min(maxLimit, value)),
               std::memory_order_relaxed);
        }
        prevState = currState;
        xSemaphoreGive(mutex);
    }

    int8_t get() {
        return rotation.load(std::memory_order_relaxed);
    }

private:
    int8_t minLimit;
    int8_t maxLimit;
    std::atomic<int8_t> rotation;
    uint8_t prevState;
    SemaphoreHandle_t mutex;
};
Knob knob0(0, 8);
Knob knob1(0, 8);
Knob knob2(0, 8);
Knob knob3(0, 8);

class Joystick {
public:

  enum Direction {
    CENTER, NORTH, SOUTH, EAST, WEST
  };

  Joystick(uint8_t xPin, uint8_t yPin)
    : _xPin(xPin), _yPin(yPin) {}

  void update(uint8_t pressed) {

    int x = analogRead(_xPin);
    int y = analogRead(_yPin);

    _pressed.store(pressed, std::memory_order_relaxed);

    determineDirection(x, y);
  }

  Direction get() const {
    return _dir.load(std::memory_order_relaxed);
  }

  bool pressed() const {
    return _pressed.load(std::memory_order_relaxed);
  }

private:
  uint8_t _xPin, _yPin;

  std::atomic<bool> _pressed{false};
  std::atomic<Direction> _dir{CENTER};

  const int DEADZONE = 100;

  void determineDirection(int x, int y) {

    int dx = x - 512;
    int dy = y - 512;

    Direction newDir;

    if (abs(dx) < DEADZONE && abs(dy) < DEADZONE)
      newDir = CENTER;
    else if (abs(dx) > abs(dy))
      newDir = (dx > 0) ? WEST : EAST;
    else
      newDir = (dy > 0) ? NORTH : SOUTH;

    _dir.store(newDir, std::memory_order_relaxed);
  }
};
Joystick joystick(JOYX_PIN, JOYY_PIN);

struct {
  std::bitset<32> inputs;
  uint8_t RX_Message[11];
  SemaphoreHandle_t mutex;
  SemaphoreHandle_t rx_message_mutex;
  SemaphoreHandle_t tx_message_mutex;
} sysState;

QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    sysState.inputs.reset();
    for (int i=0; i<8; i++){
      setRow(i);
      delayMicroseconds(3);
      sysState.inputs |= (std::bitset<32>(readCols().to_ulong())) << 4*i;
    }

    knob0.update(sysState.inputs[18], sysState.inputs[19]);
    knob1.update(sysState.inputs[16], sysState.inputs[17]);
    knob2.update(sysState.inputs[14], sysState.inputs[15]);
    knob3.update(sysState.inputs[12], sysState.inputs[13]);

    uint8_t currentStepNumbers[NUM_VOICES] = {0};
    int8_t keysPressed = 0;
    for (int i=0; i<12; i++){
      if (sysState.inputs[i] == 0 and keysPressed < NUM_VOICES){
        currentStepNumbers[keysPressed] = i;
        keysPressed++;
      }
    }

    uint8_t TX_Message[11] = {0};
    if (keysPressed == 0) {
      TX_Message[0] = 'R';
    } else {        
      TX_Message[0] = 'P';

      // If a key is pressed, transmit the int value
      // Else transmit 0xFF
      for (int i=0; i<keysPressed; i++)
        TX_Message[i+2] = currentStepNumbers[i];
      for (int i=keysPressed; i<NUM_VOICES; i++)
        TX_Message[i+2] = 0xFF;

    }
    TX_Message[1] = 4;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
}

void scanJoystickTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    setRow(5);
    delayMicroseconds(3);
    joystick.update(readCols()[2]);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
    u8g2.clearBuffer();                  // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font

    u8g2.setCursor(0,0);

    const char* waveLabels[4] = { "SAW", "SIN", "TRI", "SQR" };
    static uint8_t selectedWave = 0;
    Joystick::Direction dir = joystick.get();
    static Joystick::Direction lastDir = Joystick::CENTER;

    // Only update selection when direction changes
    if (dir != lastDir) {
        if (dir == Joystick::EAST) {                // move right
            selectedWave = (selectedWave + 1) % 4;
        } else if (dir == Joystick::WEST) {         // move left
            selectedWave = (selectedWave + 3) % 4;  // wrap around
        }
    }
    lastDir = dir;

    // Display waveforms in a single line
    u8g2.setFont(u8g2_font_ncenB08_tr);
    uint8_t x = 0;           // start X
    uint8_t y = 10;          // Y position
    uint8_t spacing = 6;     // pixels between words

    for (uint8_t i = 0; i < 4; i++) {

        uint8_t w = u8g2.getStrWidth(waveLabels[i]);

        if (i == selectedWave) {
            // highlight background
            uint8_t h = 10;  // approximate font height
            u8g2.drawBox(x, y - h + 2, w, h);
            u8g2.setDrawColor(0);  // text color black
        } else {
            u8g2.setDrawColor(1);  // normal white text
        }

        u8g2.setCursor(x, y);
        u8g2.print(waveLabels[i]);

        // reset draw color for next
        u8g2.setDrawColor(1);

        x += w + spacing;  // move to next word
    }

    currentWaveState.store(selectedWave, std::memory_order_release);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.setCursor(0,20);
    u8g2.print(knob3.get(),DEC); 
    xSemaphoreGive(sysState.mutex);

    u8g2.setCursor(20,30);

    uint8_t localRXMessage[11] = {0};
    xSemaphoreTake(sysState.rx_message_mutex, portMAX_DELAY);
    memcpy(localRXMessage, sysState.RX_Message, 11);
    xSemaphoreGive(sysState.rx_message_mutex);

    if (localRXMessage[0] == 'R'){
      u8g2.print('R');
    } else {
      u8g2.print('P');
      u8g2.print(' ');
      u8g2.print(localRXMessage[1]);
      u8g2.print(' ');
      for (int i=0; i<NUM_VOICES; i++){
        if (localRXMessage[i+2] != 0xFF) {
          u8g2.print(localRXMessage[i+2]);
          u8g2.print(' ');
        } else {
          break;
        }
      }
    }

    u8g2.sendBuffer();                   // transfer internal memory to the display

    digitalToggle(LED_BUILTIN);          // Toggle LED
  }
}

void transmitTask (void * pvParameters) {
	uint8_t msgOut[11];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(sysState.tx_message_mutex, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void decodeTask(void * pvParameters) {

  uint8_t localRXMessage[11] = {0};

  while (true) {
    xQueueReceive(msgInQ, localRXMessage, portMAX_DELAY);

    xSemaphoreTake(sysState.rx_message_mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, localRXMessage, 8);
    xSemaphoreGive(sysState.rx_message_mutex);

    uint32_t localCurrentStepSize = 0;
    if (localRXMessage[0] == 'P') {
      uint8_t octaveNumber = localRXMessage[1];

      for (int i=0; i<NUM_VOICES; i++){
        uint8_t localCurrentStepNumber = localRXMessage[i+2];
        if (localCurrentStepNumber != 0xFF) {
          localCurrentStepSize = stepSizes[localCurrentStepNumber] << (octaveNumber - 4);
          __atomic_store_n(&currentStepSize[i], localCurrentStepSize, __ATOMIC_RELAXED);
        }
      }
    }

    else {
      for (int i=0; i<NUM_VOICES; i++) {
        __atomic_store_n(&currentStepSize[i], 0, __ATOMIC_RELAXED);
      }
    }

  }
}

inline int32_t renderWave(uint32_t phase, uint8_t wavetype) {

  switch (wavetype) {

    case WAVE_SAW:
      return (phase >> 24) - 128;

    case WAVE_SQUARE:
      return (phase & 0x80000000) ? 127 : -128;

    case WAVE_TRIANGLE: {
      uint8_t ramp = phase >> 24;
      if (ramp < 128)
        return (ramp << 1) - 128;
      else
        return ((255 - ramp) << 1) - 128;
    }

    case WAVE_SINE:
      return (sineTable[phase >> 24] - 128);
  }

  return 0;
}

void sampleISR() {
  static uint32_t phaseAcc[NUM_VOICES];
  int32_t mixedSound = 0;
  uint8_t localCurrentWaveState = currentWaveState.load(std::memory_order_acquire);

  for (int i = 0; i < NUM_VOICES; i++) {
    if (currentStepSize[i]) {
      phaseAcc[i] += currentStepSize[i];
      mixedSound += renderWave(phaseAcc[i], localCurrentWaveState);
    }
  }

  // Divide sound across voices equally
  mixedSound = mixedSound / NUM_VOICES;

  // Apply low-pass filter ONLY for sine wave
  static float prevFiltered = 0.0f;
  if (localCurrentWaveState == WAVE_SINE) {
    // Convert to float -1.0 .. 1.0 for filtering
    float normalized = ((float)mixedSound / 128.0f) - 1.0f;

    // One-pole IIR filter
    const float alpha = 0.05f;  // adjust for cutoff frequency
    float filtered = prevFiltered + alpha * (normalized - prevFiltered);
    prevFiltered = filtered;

    // Convert back to int for PWM
    mixedSound = (int32_t)((filtered + 1.0f) * 128.0f);

    mixedSound = mixedSound << SINE_FACTOR;
  }

  mixedSound = mixedSound >> (8 - knob3.get());
  analogWrite(OUTR_PIN, mixedSound + 128);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(sysState.tx_message_mutex, NULL);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setup() {
  // put your setup code here, to run once:
  initStepSizes();
  initSineTable();

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  sysState.mutex = xSemaphoreCreateMutex();
  sysState.rx_message_mutex = xSemaphoreCreateMutex();
  sysState.tx_message_mutex = xSemaphoreCreateCounting(3,3);

  TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
    scanKeysTask,		    /* Function that implements the task */
    "scanKeys",		      /* Text name for the task */
    64,      		        /* Stack size in words, not bytes */
    NULL,			          /* Parameter passed into the task */
    2,			            /* Task priority */
    &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t scanJoystickHandle = NULL;
    xTaskCreate(
    scanJoystickTask,		    
    "scanJoystick",
    64,      		    
    NULL,			       
    1,			           
    &scanJoystickHandle );

  TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(
    displayUpdateTask,		    
    "displayUpdate",
    256,      		    
    NULL,			       
    1,			           
    &displayUpdateHandle );

  TaskHandle_t decodeHandle = NULL;
    xTaskCreate(
    decodeTask,		    
    "decode",
    256,      		    
    NULL,			       
    1,			           
    &decodeHandle );

  TaskHandle_t transmitHandle = NULL;
    xTaskCreate(
    transmitTask,		    
    "transmit",
    256,      		    
    NULL,			       
    1,			           
    &transmitHandle );

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
  setOutMuxBit(KNOB_MODE, HIGH);  //Read knobs through key matrix

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  vTaskStartScheduler();
}

void loop() {
  
}