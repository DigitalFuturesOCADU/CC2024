#include <ArduinoBLE.h>

#define DEVICE_NAME "ButtonDevice_A"  // Change A to B,C,etc for different devices

// Simplified UUIDs - each with a unique last segment
const char* SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1000";
const char* BUTTON1_UUID = "19B10000-E8F2-537E-4F6C-D104768A1001";
const char* BUTTON2_UUID = "19B10000-E8F2-537E-4F6C-D104768A1002";
const char* BUTTON3_UUID = "19B10000-E8F2-537E-4F6C-D104768A1003";
const char* BUTTON4_UUID = "19B10000-E8F2-537E-4F6C-D104768A1004";
const char* BUTTON5_UUID = "19B10000-E8F2-537E-4F6C-D104768A1005";

// Array of UUIDs for easy reference
const char* BUTTON_UUIDS[] = {
    BUTTON1_UUID, 
    BUTTON2_UUID, 
    BUTTON3_UUID, 
    BUTTON4_UUID, 
    BUTTON5_UUID
};

// Create BLE service
BLEService buttonService(SERVICE_UUID);

// Array of characteristics
BLEBoolCharacteristic* buttonCharacteristics[] = {
    new BLEBoolCharacteristic(BUTTON1_UUID, BLERead | BLENotify),
    new BLEBoolCharacteristic(BUTTON2_UUID, BLERead | BLENotify),
    new BLEBoolCharacteristic(BUTTON3_UUID, BLERead | BLENotify),
    new BLEBoolCharacteristic(BUTTON4_UUID, BLERead | BLENotify),
    new BLEBoolCharacteristic(BUTTON5_UUID, BLERead | BLENotify)
};

// Pin definitions
const int BUTTON_PINS[] = {2, 3, 4, 5, 6};  // Buttons
const int LED_PINS[] = {8, 9, 10, 11, 12};  // LEDs
const int NUM_BUTTONS = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);
const int BUILTIN_LED = LED_BUILTIN;

// Debounce settings
const unsigned long DEBOUNCE_TIME = 50;
unsigned long* lastUpdateTime = new unsigned long[NUM_BUTTONS]();
bool* lastButtonState = new bool[NUM_BUTTONS]();

// Connection status variables
bool isConnected = false;
unsigned long lastBlinkTime = 0;
const long BLINK_INTERVAL = 1000;
bool ledState = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for serial connection
  
  Serial.println("BLE Button Device Starting...");
  
  // Initialize pins
  for(int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    pinMode(LED_PINS[i], OUTPUT);
    Serial.print("Initialized button ");
    Serial.print(i + 1);
    Serial.print(" on pin ");
    Serial.println(BUTTON_PINS[i]);
  }
  pinMode(BUILTIN_LED, OUTPUT);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1) {
      digitalWrite(BUILTIN_LED, HIGH);
      delay(100);
      digitalWrite(BUILTIN_LED, LOW);
      delay(100);
    }
  }

  // Set up BLE
  BLE.setLocalName(DEVICE_NAME);
  BLE.setAdvertisedService(buttonService);
  
  // Add characteristics
  for(int i = 0; i < NUM_BUTTONS; i++) {
    buttonService.addCharacteristic(*buttonCharacteristics[i]);
    Serial.print("Added characteristic for button ");
    Serial.println(i + 1);
  }
  
  BLE.addService(buttonService);
  BLE.setEventHandler(BLEConnected, onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  BLE.poll();
  unsigned long currentTime = millis();

  updateConnectionLED(currentTime);
  
  if (isConnected) {
    for(int i = 0; i < NUM_BUTTONS; i++) {
      bool currentButtonState = !digitalRead(BUTTON_PINS[i]);
      
      if (currentTime - lastUpdateTime[i] > DEBOUNCE_TIME) {
        if (currentButtonState != lastButtonState[i]) {
          lastUpdateTime[i] = currentTime;
          lastButtonState[i] = currentButtonState;
          
          buttonCharacteristics[i]->writeValue(currentButtonState);
          Serial.print("Button ");
          Serial.print(i + 1);
          Serial.print(" state changed to: ");
          Serial.println(currentButtonState ? "PRESSED" : "RELEASED");
        }
      }
    }
  }
}

void onBLEConnected(BLEDevice central) {
  isConnected = true;
  digitalWrite(BUILTIN_LED, HIGH);
  Serial.print("Connected to central MAC: ");
  Serial.println(central.address());
}

void onBLEDisconnected(BLEDevice central) {
  isConnected = false;
  digitalWrite(BUILTIN_LED, LOW);
  Serial.print("Disconnected from central MAC: ");
  Serial.println(central.address());
}

void updateConnectionLED(unsigned long currentTime) {
  if (!isConnected) {
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(BUILTIN_LED, ledState);
    }
  }
}