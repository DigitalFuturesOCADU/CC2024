// combined_sensors.ino

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>
#include <Arduino_LSM6DS3.h>
#include "SensorFusion.h"

// Pin definitions
#define CAP1188_RESET  12
#define POT_PIN A6
#define LIGHT_PIN A7

// Averaging window sizes
#define POT_AVERAGE_WINDOW 10
#define LIGHT_AVERAGE_WINDOW 10

// Create sensor objects
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);
SF fusion;

// Global variables for touch sensor
bool touchStates[8] = {false};
unsigned long lastTouchReadTime = 0;
unsigned int touchReadInterval = 50;

// Global variables for potentiometer
int potValue = 0;
int smoothedPotValue = 0;
unsigned long lastPotReadTime = 0;
unsigned int potReadInterval = 50;

// Global variables for light sensor
int lightValue = 0;
int smoothedLightValue = 0;
unsigned long lastLightReadTime = 0;
unsigned int lightReadInterval = 50;

// Global variables for IMU
float gx, gy, gz, ax, ay, az;
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
float deltat = 0.0f;
unsigned long lastImuReadTime = 0;
unsigned int imuReadInterval = 10;  // 100Hz update rate

// Calibration variables for IMU
float gyro_bias[3] = {0, 0, 0};
const int calibration_samples = 1000;

// Rolling average variables for potentiometer
int potReadings[POT_AVERAGE_WINDOW];
int potReadIndex = 0;
long potTotalValue = 0;

// Rolling average variables for light sensor
int lightReadings[LIGHT_AVERAGE_WINDOW];
int lightReadIndex = 0;
long lightTotalValue = 0;

// IMU Calibration function
void calibrateImu() {
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  int valid_samples = 0;
  
  Serial.println("Keep the IMU still for calibration...");
  delay(2000);  // Give user time to place IMU still
  
  Serial.println("Calibrating...");
  
  // Collect samples
  while (valid_samples < calibration_samples) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // Accumulate gyro readings
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
      
      // Accumulate accel readings
      sum_ax += ax;
      sum_ay += ay;
      sum_az += az;
      
      valid_samples++;
      
      // Show progress every 100 samples
      if (valid_samples % 100 == 0) {
        Serial.print("Progress: ");
        Serial.print((valid_samples * 100) / calibration_samples);
        Serial.println("%");
      }
      
      delay(2);  // Small delay between readings
    }
  }
  
  // Calculate average gyro bias
  gyro_bias[0] = sum_gx / calibration_samples;
  gyro_bias[1] = sum_gy / calibration_samples;
  gyro_bias[2] = sum_gz / calibration_samples;
  
  // Calculate initial orientation from average accelerometer readings
  float initial_ax = sum_ax / calibration_samples;
  float initial_ay = sum_ay / calibration_samples;
  float initial_az = sum_az / calibration_samples;
  
  // Update fusion filter with initial values
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(0, 0, 0, initial_ax, initial_ay, initial_az, deltat);
  
  Serial.println("Calibration complete!");
  Serial.println("Gyro bias values:");
  Serial.print("X: "); Serial.print(gyro_bias[0]);
  Serial.print(" Y: "); Serial.print(gyro_bias[1]);
  Serial.print(" Z: "); Serial.println(gyro_bias[2]);
}

// Initialization functions
bool initializeTouchSensor() {
  return touchInput.begin();
}

bool initializeImu() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return false;
  }
  
  calibrateImu();  // Perform calibration during initialization
  return true;
}

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

// Sensor read functions
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

void readPotentiometer() {
  unsigned long currentTime = millis();
  if (currentTime - lastPotReadTime >= potReadInterval) {
    potValue = analogRead(POT_PIN);
    updatePotAverage(potValue);
    printPotValue();
    lastPotReadTime = currentTime;
  }
}

void readLightSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastLightReadTime >= lightReadInterval) {
    lightValue = analogRead(LIGHT_PIN);
    updateLightAverage(lightValue);
    printLightValue();
    lastLightReadTime = currentTime;
  }
}

void readImu() {
  unsigned long currentTime = millis();
  if (currentTime - lastImuReadTime >= imuReadInterval) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      // Read acceleration and gyroscope data
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // Remove bias from gyro readings
      gx -= gyro_bias[0];
      gy -= gyro_bias[1];
      gz -= gyro_bias[2];
      
      // Convert gyroscope readings from deg/s to rad/s
      gx *= DEG_TO_RAD;
      gy *= DEG_TO_RAD;
      gz *= DEG_TO_RAD;
      
      // Update the filter
      deltat = fusion.deltatUpdate();
      fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
      
      // Get the angles
      pitch = fusion.getPitch();
      roll = fusion.getRoll();
      yaw = fusion.getYaw();
      
      printImuValues();
    }
    lastImuReadTime = currentTime;
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

void printImuValues() {
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.print("\tPitch: ");
  Serial.print(pitch, 1);
  Serial.print("\tYaw: ");
  Serial.println(yaw, 1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Combined sensors with IMU test!");

  // Initialize all sensors
  if (!initializeTouchSensor()) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("Touch sensor initialized!");
  
  if (!initializeImu()) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  Serial.println("IMU initialized and calibrated!");
  
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
  readImu();
}