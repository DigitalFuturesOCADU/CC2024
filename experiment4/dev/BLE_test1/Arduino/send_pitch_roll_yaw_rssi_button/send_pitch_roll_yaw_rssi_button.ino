/*******************************************************************************
 * Arduino BLE Orientation and Button Sensor
 * 
 * This program reads orientation data from an IMU (Inertial Measurement Unit) 
 * and a button state, then sends this information over Bluetooth Low Energy (BLE).
 * 
 * Hardware Required:
 * - Arduino Nano 33 BLE or similar
 * - Push button connected to D3 and GND
 * 
 * Libraries Required:
 * - ArduinoBLE: Handles Bluetooth Low Energy communication
 * - Arduino_LSM6DS3: Interfaces with the onboard IMU sensor
 * - SensorFusion: Combines accelerometer and gyroscope data
 * 
 * Key Concepts:
 * 
 * 1. BLE (Bluetooth Low Energy):
 *    - Service: A collection of related data (like a folder)
 *    - Characteristics: Individual pieces of data within a service (like files)
 *    - UUID: Unique identifiers for services and characteristics
 *    - Notify: Sends data when it changes (instead of waiting for requests)
 * 
 * 2. IMU (Inertial Measurement Unit):
 *    - Accelerometer: Measures acceleration and gravity
 *    - Gyroscope: Measures rotation rates
 *    - Fusion: Combines both sensors for accurate orientation
 *    - Euler Angles: Pitch (x), Roll (y), and Yaw (z) rotation
 * 
 * 3. Button Input:
 *    - INPUT_PULLUP: Internal resistor pulls pin HIGH when button is not pressed
 *    - Debouncing: Removes false triggers from button noise
 * 
 * Code Structure:
 * - setupBLE(): Configure BLE service and characteristics
 * - setupHardware(): Initialize IMU and button
 * - handleButton(): Process button input with debouncing
 * - updateOrientation(): Read and process IMU data
 * - updateRSSI(): Update signal strength
 * - handleConnection(): Manage BLE connection state
 * - blinkLED(): Status indicator
 ******************************************************************************/

#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "SensorFusion.h"

// BLE configuration
const char* DEVICE_NAME = "OrientationSensor";
const char* SERVICE_UUID = "19B10010-E8F2-537E-4F6C-D104768A1214";
const char* PITCH_CHAR_UUID = "19B10012-E8F2-537E-4F6C-D104768A1214";
const char* ROLL_CHAR_UUID = "19B10013-E8F2-537E-4F6C-D104768A1214";
const char* YAW_CHAR_UUID = "19B10014-E8F2-537E-4F6C-D104768A1214";
const char* RSSI_CHAR_UUID = "19B10015-E8F2-537E-4F6C-D104768A1214";
const char* BUTTON_CHAR_UUID = "19B10016-E8F2-537E-4F6C-D104768A1214";

// Pin definitions
const int BUTTON_PIN = 3;

// BLE service and characteristics
BLEService orientationService(SERVICE_UUID);
BLEFloatCharacteristic pitchCharacteristic(PITCH_CHAR_UUID, BLENotify);
BLEFloatCharacteristic rollCharacteristic(ROLL_CHAR_UUID, BLENotify);
BLEFloatCharacteristic yawCharacteristic(YAW_CHAR_UUID, BLENotify);
BLEIntCharacteristic rssiCharacteristic(RSSI_CHAR_UUID, BLENotify);
BLEBoolCharacteristic buttonCharacteristic(BUTTON_CHAR_UUID, BLENotify);

// Sensor fusion setup
SF fusion;
float gx, gy, gz;
float ax, ay, az;
float pitch, roll, yaw;
float deltat;

// Button state tracking
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastButtonDebounce = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Timing control
unsigned long lastUpdate = 0;
unsigned long lastRSSIUpdate = 0;
unsigned long lastBlink = 0;
const unsigned long UPDATE_INTERVAL = 50;    // 20Hz update rate
const unsigned long RSSI_INTERVAL = 1000;    // 1Hz RSSI update
const unsigned long BLINK_INTERVAL = 1000;
bool ledState = false;

/**
 * Initialize hardware components (IMU and button)
 * Returns true if successful, false if IMU fails
 */
bool setupHardware() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  return IMU.begin();
}

/**
 * Initialize and configure BLE
 * Returns true if successful, false if BLE fails
 */
bool setupBLE() {
  if (!BLE.begin()) return false;
  
  BLE.setLocalName(DEVICE_NAME);
  BLE.setAdvertisedService(orientationService);
  
  // Add all characteristics
  orientationService.addCharacteristic(pitchCharacteristic);
  orientationService.addCharacteristic(rollCharacteristic);
  orientationService.addCharacteristic(yawCharacteristic);
  orientationService.addCharacteristic(rssiCharacteristic);
  orientationService.addCharacteristic(buttonCharacteristic);
  
  BLE.addService(orientationService);
  BLE.advertise();
  
  return true;
}

/**
 * Handle button input with debouncing
 * Returns true if button state changed
 */
bool handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  bool stateChanged = false;
  
  if (reading != lastButtonState) {
    lastButtonDebounce = millis();
  }
  
  if ((millis() - lastButtonDebounce) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      buttonCharacteristic.writeValue(!buttonState);  // Invert because of INPUT_PULLUP
      stateChanged = true;
      Serial.print("Button: "); Serial.println(!buttonState);
    }
  }
  
  lastButtonState = reading;
  return stateChanged;
}

/**
 * Update orientation data from IMU
 * Returns true if new data was processed
 */
bool updateOrientation() {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return false;
  
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  
  // Convert gyroscope data to radians
  gx = gx * DEG_TO_RAD;
  gy = gy * DEG_TO_RAD;
  gz = gz * DEG_TO_RAD;
  
  // Update sensor fusion
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
  
  // Get orientation angles
  pitch = fusion.getPitch();
  roll = fusion.getRoll();
  yaw = fusion.getYaw();
  
  // Send data over BLE
  pitchCharacteristic.writeValue(pitch);
  rollCharacteristic.writeValue(roll);
  yawCharacteristic.writeValue(yaw);
  
  // Debug output
  Serial.print("P: "); Serial.print(pitch);
  Serial.print(" R: "); Serial.print(roll);
  Serial.print(" Y: "); Serial.println(yaw);
  
  return true;
}

/**
 * Update RSSI value
 * Returns true if RSSI was updated
 */
bool updateRSSI() {
  int rssi = BLE.rssi();
  if (rssi != 0) {
    rssiCharacteristic.writeValue(rssi);
    return true;
  }
  return false;
}

/**
 * Blink LED for status indication
 */
void blinkLED() {
  if (millis() - lastBlink >= BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlink = millis();
  }
}

/**
 * Handle connected device
 * Returns true while device remains connected
 */
bool handleConnection(BLEDevice central) {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Connected to central");
  
  while (central.connected()) {
    // Handle button input
    handleButton();

    // Update orientation data at specified interval
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
      updateOrientation();
      lastUpdate = millis();
    }
    
    // Update RSSI at specified interval
    if (millis() - lastRSSIUpdate >= RSSI_INTERVAL) {
      updateRSSI();
      lastRSSIUpdate = millis();
    }
    
    return true;
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Disconnected from central");
  return false;
}

void setup() {
  Serial.begin(9600);
  
  // Initialize hardware
  if (!setupHardware()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  // Initialize BLE
  if (!setupBLE()) {
    Serial.println("Failed to initialize BLE!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  Serial.println("BLE Orientation Sensor Active");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    handleConnection(central);
  } else {
    blinkLED();
  }
}