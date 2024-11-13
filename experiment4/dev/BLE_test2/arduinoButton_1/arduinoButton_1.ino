/*******************************************************************************
 * Arduino BLE Button/LED Peripheral - DEVICE 1
 * Each Arduino needs a unique name for identification
 ******************************************************************************/

#include <ArduinoBLE.h>

// BLE configuration
const char* DEVICE_NAME = "ButtonLED1";  // UNIQUE name for this device!
const char* SERVICE_UUID = "19B10010-E8F2-537E-4F6C-D104768A1214";
const char* BUTTON_CHAR_UUID = "19B10011-E8F2-537E-4F6C-D104768A1214";
const char* LED_CHAR_UUID = "19B10012-E8F2-537E-4F6C-D104768A1214";

// Pin definitions
const int BUTTON_PIN = 3;
const int LED_PIN = 5;

// BLE service and characteristics
BLEService buttonLEDService(SERVICE_UUID);
BLEBoolCharacteristic buttonCharacteristic(BUTTON_CHAR_UUID, BLENotify);
BLEBoolCharacteristic ledCharacteristic(LED_CHAR_UUID, BLEWrite);

// Button state tracking
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastButtonDebounce = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void setup() {
  Serial.begin(9600);
  
  // Setup pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // Setup BLE
  BLE.setLocalName(DEVICE_NAME);  // This sets the unique name
  BLE.setAdvertisedService(buttonLEDService);
  
  buttonLEDService.addCharacteristic(buttonCharacteristic);
  buttonLEDService.addCharacteristic(ledCharacteristic);
  
  BLE.addService(buttonLEDService);
  buttonCharacteristic.writeValue(false);
  
  // Start advertising
  BLE.advertise();
  Serial.println("BLE Button/LED Peripheral 1 Active");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.println("Connected to central");
    digitalWrite(LED_BUILTIN, HIGH);
    
    while (central.connected()) {
      // Handle button
      bool reading = digitalRead(BUTTON_PIN);
      
      if (reading != lastButtonState) {
        lastButtonDebounce = millis();
      }
      
      if ((millis() - lastButtonDebounce) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
          buttonState = reading;
          buttonCharacteristic.writeValue(!buttonState);
          Serial.print("Button: "); Serial.println(!buttonState);
        }
      }
      lastButtonState = reading;
      
      // Handle LED
      if (ledCharacteristic.written()) {
        bool value = ledCharacteristic.value();
        digitalWrite(LED_PIN, value);
        Serial.print("LED set to: "); Serial.println(value);
      }
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected");
  }
}