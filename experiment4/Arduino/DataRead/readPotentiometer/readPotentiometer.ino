// potentiometer_module.ino

#define POT_PIN A6
#define AVERAGE_WINDOW 12  // Number of samples to average

// Global variables
int potValue = 0;          // Raw value
int smoothedValue = 0;     // Filtered value
unsigned long lastPotReadTime = 0;
unsigned int potReadInterval = 20;  // Time between reads in milliseconds

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

// Function to read potentiometer and update the global value
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

// Function to print potentiometer values
void printPotValue() {

  Serial.print("\tPot Smoothed: ");
  Serial.println(smoothedValue);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Potentiometer test!");
  
  // Set pin mode
  pinMode(POT_PIN, INPUT);
  
  // Initialize the rolling average
  initializeRollingAverage();
}

void loop() {
  // Read the potentiometer (function handles timing internally)
  readPotentiometer();
}