/*******************************************************************************
 * Light Sensor Control System
 * 
 * HARDWARE CONFIGURATION:
 * - Analog light sensor connected to pin A7
 * 
 * CONTROL VARIABLES:
 * Pin Configuration:
 * - lightPin (A7)       : Analog input pin for light sensor
 * 
 * Timing Controls:
 * - lightReadInterval   : Milliseconds between sensor readings (50ms)
 * - lastLightReadTime   : Timestamp of last sensor reading
 * 
 * Rolling Average Configuration:
 * - lightAverageWindow  : Number of samples in rolling average (10)
 * - lightReadings[]     : Array storing historical readings
 * - lightReadIndex      : Current position in rolling average array
 * - lightTotalValue     : Running sum of all readings
 * 
 * Sensor Values:
 * - lightValue          : Raw analog reading (0-1023)
 * - smoothedLightValue  : Filtered value from rolling average
 * 
 * FUNCTIONALITY:
 * - Performs analog readings at specified intervals
 * - Implements rolling average for noise reduction
 * - Outputs both raw and smoothed values via Serial
 * 
 * Serial Output Format:
 * "Light Raw: [value]\tLight Smoothed: [value]"
 * 
 *******************************************************************************/
 

int lightPin = A7;
const int lightAverageWindow = 10; // Number of samples to average



// Global variables
int lightValue = 0;          // Raw value
int smoothedLightValue = 0;  // Filtered value
unsigned long lastLightReadTime = 0;
unsigned int lightReadInterval = 50;  // Time between reads in milliseconds

// Rolling average variables
int lightReadings[lightAverageWindow];
int lightReadIndex = 0;
long lightTotalValue = 0;

// Function to initialize the rolling average array
void initializeLightAverage() {
  // Initialize all readings to 0
  for (int i = 0; i < lightAverageWindow; i++) {
    lightReadings[i] = 0;
  }
  lightTotalValue = 0;
  lightReadIndex = 0;
}

// Function to update rolling average with new value
void updateLightAverage(int newValue) {
  // Subtract the oldest reading from the total
  lightTotalValue = lightTotalValue - lightReadings[lightReadIndex];
  // Add the new reading to the array
  lightReadings[lightReadIndex] = newValue;
  // Add the new reading to the total
  lightTotalValue = lightTotalValue + newValue;
  // Advance to the next position in the array
  lightReadIndex = (lightReadIndex + 1) % lightAverageWindow;
  // Calculate the average
  smoothedLightValue = lightTotalValue / lightAverageWindow;
}

// Function to read light sensor and update the global value
void readLightSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastLightReadTime >= lightReadInterval) {
    // Read the analog value
    lightValue = analogRead(lightPin);
    
    // Update the rolling average
    updateLightAverage(lightValue);
    
    // Print the values
    printLightValue();
    
    // Update the last read time
    lastLightReadTime = currentTime;
  }
}

// Function to print light sensor values
void printLightValue() {
  Serial.print("Light Raw: ");
  Serial.print(lightValue);
  Serial.print("\tLight Smoothed: ");
  Serial.println(smoothedLightValue);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Light sensor");
  

  
  // Initialize the rolling average
  initializeLightAverage();
}

void loop() {
  // Read the light sensor (function handles timing internally)
  readLightSensor();
}