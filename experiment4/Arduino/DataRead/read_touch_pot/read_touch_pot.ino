// combined_sensors.ino

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

#define CAP1188_RESET  12
#define POT_PIN A6
#define AVERAGE_WINDOW 10  // Number of samples to average

// Create the touch sensor object with reset pin
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);

// Global variables for touch sensor
bool touchStates[8] = {false};
unsigned long lastReadTime = 0;
unsigned int readInterval = 50;  // Time between reads in milliseconds

// Global variables for potentiometer
int potValue = 0;          // Raw value
int smoothedValue = 0;     // Filtered value
unsigned long lastPotReadTime = 0;
unsigned int potReadInterval = 50;  // Time between reads in milliseconds

// Rolling average variables
int readings[AVERAGE_WINDOW];
int readIndex = 0;
long totalValue = 0;

// Function to initialize the rolling average array
void initializeRollingAverage() {
  // Initialize all readings to 0
  for (int i = 0; i < AVERAGE_WINDOW; i++) {
    readings[i] = 0;
  }
  totalValue = 0;
  readIndex = 0;
}

// Function to update rolling average with new value
void updateRollingAverage(int newValue) {
  // Subtract the oldest reading from the total
  totalValue = totalValue - readings[readIndex];
  // Add the new reading to the array
  readings[readIndex] = newValue;
  // Add the new reading to the total
  totalValue = totalValue + newValue;
  // Advance to the next position in the array
  readIndex = (readIndex + 1) % AVERAGE_WINDOW;
  // Calculate the average
  smoothedValue = totalValue / AVERAGE_WINDOW;
}

// Touch sensor functions
void readTouchInputs() {
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval) {
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

bool initializeTouchSensor() {
  return touchInput.begin();
}

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

// Potentiometer functions
void readPotentiometer() {
  unsigned long currentTime = millis();
  if (currentTime - lastPotReadTime >= potReadInterval) {
    // Read the analog value
    potValue = analogRead(POT_PIN);
    
    // Update the rolling average
    updateRollingAverage(potValue);
    
    // Print the values
    printPotValue();
    
    // Update the last read time
    lastPotReadTime = currentTime;
  }
}

void printPotValue() {
  Serial.print("Pot Raw: ");
  Serial.print(potValue);
  Serial.print("\tPot Smoothed: ");
  Serial.println(smoothedValue);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Combined sensors test!");

  // Initialize touch sensor
  if (!initializeTouchSensor()) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
  
  // Initialize potentiometer
  pinMode(POT_PIN, INPUT);
  initializeRollingAverage();
}

void loop() {
  // Read both sensors (functions handle timing internally)
  readTouchInputs();
  readPotentiometer();
}