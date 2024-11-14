/**
 * CAP1188 Touch Sensor Handler
 * 
 * Manages and tracks capacitive touch inputs from the Adafruit CAP1188 sensor.
 * Monitors 8 touch channels with current and previous state tracking for edge detection.
 * 
 * Key Components:
 * @hardware Adafruit CAP1188 capacitive touch sensor
 * @pin CAP1188_RESET - Pin 12, used for hardware reset
 * 
 * Key Variables:
 * @var touchStates[8] - Current state of each touch input (true/false)
 * @var prevTouchStates[8] - Previous state of each input for edge detection
 * @var readInterval - Time between sensor reads (50ms default)
 * @var lastReadTime - Timestamp of last sensor read
 * 
 * Communication:
 * - Uses I2C for sensor communication
 * - Serial output at 9600 baud for debugging
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

#define CAP1188_RESET  12

// Create the touch sensor object with reset pin
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);

// Global variables
bool touchStates[8] = {false};
bool prevTouchStates[8] = {false};  // previous states array
unsigned long lastReadTime = 0;
unsigned int readInterval = 50;  // Time between reads in milliseconds (can be modified)

// Function to read all 8 touch inputs and update the global touchStates array
void readTouchInputs() {
  // Only read if enough time has passed
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
    
    // Print the states
    printTouchStates();
    
    // Update the last read time
    lastReadTime = currentTime;
  }
}

// Function to check if a specific touch input just became active
bool isTouchStarted(uint8_t touchIndex) {
  return touchStates[touchIndex] && !prevTouchStates[touchIndex];
}

// Function to check if a specific touch input just became inactive
bool isTouchEnded(uint8_t touchIndex) {
  return !touchStates[touchIndex] && prevTouchStates[touchIndex];
}

// Function to detect and report edge transitions for all touch inputs
void checkEdgeDetection() {
  for (uint8_t i = 0; i < 8; i++) {
    if (isTouchStarted(i)) {
      Serial.print("Touch ");
      Serial.print(i + 1);
      Serial.println(" started!");
    }
    if (isTouchEnded(i)) {
      Serial.print("Touch ");
      Serial.print(i + 1);
      Serial.println(" ended!");
    }
  }
}

// Function to initialize the touch sensor
// Returns: true if initialization successful, false otherwise
bool initializeTouchSensor() {
  return touchInput.begin();
}

// Function to print touch states to Serial (prints all states regardless of touch)
void printTouchStates() {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("C");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(touchStates[i] ? "1" : "0");
    Serial.print("\t");
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  Serial.println("CAP1188 test!");

  if (!initializeTouchSensor()) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
}

void loop() {
  // Read the touch inputs (function handles timing internally)
  readTouchInputs();
  
  // Check for edge transitions
  checkEdgeDetection();
}