// light_sensor_module.ino

#define LIGHT_PIN A7
#define LIGHT_AVERAGE_WINDOW 10  // Number of samples to average

// Global variables
int lightValue = 0;          // Raw value
int smoothedLightValue = 0;  // Filtered value
unsigned long lastLightReadTime = 0;
unsigned int lightReadInterval = 50;  // Time between reads in milliseconds

// Rolling average variables
int lightReadings[LIGHT_AVERAGE_WINDOW];
int lightReadIndex = 0;
long lightTotalValue = 0;

// Function to initialize the rolling average array
void initializeLightAverage() {
  // Initialize all readings to 0
  for (int i = 0; i < LIGHT_AVERAGE_WINDOW; i++) {
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
  lightReadIndex = (lightReadIndex + 1) % LIGHT_AVERAGE_WINDOW;
  // Calculate the average
  smoothedLightValue = lightTotalValue / LIGHT_AVERAGE_WINDOW;
}

// Function to read light sensor and update the global value
void readLightSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastLightReadTime >= lightReadInterval) {
    // Read the analog value
    lightValue = analogRead(LIGHT_PIN);
    
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
  Serial.println("Light sensor test!");
  
  // Set pin mode
  pinMode(LIGHT_PIN, INPUT);
  
  // Initialize the rolling average
  initializeLightAverage();
}

void loop() {
  // Read the light sensor (function handles timing internally)
  readLightSensor();
}