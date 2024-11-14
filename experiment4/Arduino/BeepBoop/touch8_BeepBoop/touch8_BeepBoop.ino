/**
 * CAP1188 Touch Sensor with Buzzer Control
 * 
 * Uses first two capacitive inputs to control a buzzer:
 * Touch 1 - 20ms interval beeping
 * Touch 2 - 3ms interval booping
 * 
 * Hardware:
 * - Adafruit CAP1188 capacitive touch sensor
 * - Piezo buzzer on pin 11
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

#define CAP1188_RESET  12
#define BUZZER_PIN 11

// Create the touch sensor object with reset pin
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);

// Global variables
bool touchStates[8] = {false};
bool prevTouchStates[8] = {false};
unsigned long lastReadTime = 0;
unsigned int readInterval = 50;

// Buzzer control variables
int inputState = 0;
unsigned long previousMillis = 0;
bool buzzerState = false;

void setup() {
  Serial.begin(9600);
  Serial.println("CAP1188 with Buzzer Control");

  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize touch sensor
  if (!touchInput.begin()) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
}

void loop() {
  // Read touch inputs
  readTouchInputs();
  
  // Update input state based on first two touches only
  if (touchStates[0]) {
    inputState = 1;
  } else if (touchStates[1]) {
    inputState = 2;
  } else {
    inputState = 0;
  }
  
  // Control buzzer based on input state
  buzzerOutput(inputState);
}

void readTouchInputs() {
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval) {
    // Store previous states
    for (uint8_t i = 0; i < 8; i++) {
      prevTouchStates[i] = touchStates[i];
    }
    
    // Read all touches at once
    uint8_t touched = touchInput.touched();
    
    // Convert the bit mask to individual boolean values
    for (uint8_t i = 0; i < 8; i++) {
      touchStates[i] = (touched & (1 << i)) != 0;
    }
    
    lastReadTime = currentTime;
  }
}

void buzzerOutput(int state) {
  unsigned long currentMillis = millis();
  int interval;
  
  switch(state) {
    case 1:  // Touch 1 active
      interval = 20;  // 20ms interval
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        if(buzzerState) {
          Serial.print(state);
          Serial.println(" : BEEP");
        }
      }
      break;
      
    case 2:  // Touch 2 active
      interval = 3;  // 3ms interval
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        if(buzzerState) {
          Serial.print(state);
          Serial.println(" : BOOP");
        }
      }
      break;
      
    default:  // No touch (state 0)
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
      Serial.print(state);
      Serial.println(" :(");
      break;
  }
}