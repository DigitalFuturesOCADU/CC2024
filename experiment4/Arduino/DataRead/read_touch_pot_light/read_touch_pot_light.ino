// combined_sensors.ino

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

// Pin definitions
#define CAP1188_RESET  12
#define POT_PIN A6
#define LIGHT_PIN A7

// Averaging window sizes
#define POT_AVERAGE_WINDOW 10
#define LIGHT_AVERAGE_WINDOW 10

// Create the touch sensor object with reset pin
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);

// Global variables for touch sensor
bool touchStates[8] = {false};
unsigned long lastTouchReadTime = 0;
unsigned int touchReadInterval = 50;  // Time between reads in milliseconds

// Global variables for potentiometer
int potValue = 0;          // Raw value
int smoothedPotValue = 0;  // Filtered value
unsigned long lastPotReadTime = 0;
unsigned int potReadInterval = 50;  // Time between reads in milliseconds

// Global variables for light sensor
int lightValue = 0;          // Raw value
int smoothedLightValue = 0;  // Filtered value
unsigned long lastLightReadTime = 0;
unsigned int lightReadInterval = 50;  // Time between reads in milliseconds

// Rolling average variables for potentiometer
int potReadings[POT_AVERAGE_WINDOW];
int potReadIndex = 0;
long potTotalValue = 0;

// Rolling average variables for light sensor
int lightReadings[LIGHT_AVERAGE_WINDOW];
int lightReadIndex = 0;
long lightTotalValue = 0;

// Initialization functions
void initializePotAverage() {
  for (int i = 0; i < POT_AVERAGE_WINDOW; i++) {
    potReadings[i] = 0;
  }
  potTotalValue = 0;
  potReadIndex = 0;
}

void initializeLightAverage() {
  for (int i = 0; i < LIGHT_AVERAGE_WINDOW; i++) {
    lightReadings[i] = 0;
  }
  lightTotalValue = 0;
  lightReadIndex = 0;
}

// Rolling average update functions
void updatePotAverage(int newValue) {
  potTotalValue = potTotalValue - potReadings[potReadIndex];
  potReadings[potReadIndex] = newValue;
  potTotalValue = potTotalValue + newValue;
  potReadIndex = (potReadIndex + 1) % POT_AVERAGE_WINDOW;
  smoothedPotValue = potTotalValue / POT_AVERAGE_WINDOW;
}

void updateLightAverage(int newValue) {
  lightTotalValue = lightTotalValue - lightReadings[lightReadIndex];
  lightReadings[lightReadIndex] = newValue;
  lightTotalValue = lightTotalValue + newValue;
  lightReadIndex = (lightReadIndex + 1) % LIGHT_AVERAGE_WINDOW;
  smoothedLightValue = lightTotalValue / LIGHT_AVERAGE_WINDOW;
}

// Touch sensor functions
bool initializeTouchSensor() {
  return touchInput.begin();
}

void readTouchInputs() {
  unsigned long currentTime = millis();
  if (currentTime - lastTouchReadTime >= touchReadInterval) {
    uint8_t touched = touchInput.touched();
    
    for (uint8_t i = 0; i < 8; i++) {
      touchStates[i] = (touched & (1 << i)) != 0;
    }
    
    printTouchStates();
    lastTouchReadTime = currentTime;
  }
}

// Potentiometer functions
void readPotentiometer() {
  unsigned long currentTime = millis();
  if (currentTime - lastPotReadTime >= potReadInterval) {
    potValue = analogRead(POT_PIN);
    updatePotAverage(potValue);
    printPotValue();
    lastPotReadTime = currentTime;
  }
}

// Light sensor functions
void readLightSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastLightReadTime >= lightReadInterval) {
    lightValue = analogRead(LIGHT_PIN);
    updateLightAverage(lightValue);
    printLightValue();
    lastLightReadTime = currentTime;
  }
}

// Print functions
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

void printPotValue() {
  Serial.print("Pot Raw: ");
  Serial.print(potValue);
  Serial.print("\tPot Smoothed: ");
  Serial.println(smoothedPotValue);
}

void printLightValue() {
  Serial.print("Light Raw: ");
  Serial.print(lightValue);
  Serial.print("\tLight Smoothed: ");
  Serial.println(smoothedLightValue);
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
  
  // Initialize analog sensors
  pinMode(POT_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);
  
  // Initialize rolling averages
  initializePotAverage();
  initializeLightAverage();
}

void loop() {
  // Read all sensors (functions handle timing internally)
  readTouchInputs();
  readPotentiometer();
  readLightSensor();
}