/*
analog threshold. When the value of the sensor goes above the corresponding threshold value, it lights up the led




*/

const int numInputs = 4;

// Analog input pins
const int analogInputPins[] = {A0, A1, A2, A3, A4};
// LED output pins
const int ledPins[] = {2, 3, 4, 5, 6};

// Individual thresholds for each analog input
int thresholds[] = {512, 512, 512, 512, 512};

// Boolean variables for LED states
bool ledStates[] = {false, false, false, false, false};

void setup() {
  Serial.begin(9600);
  
  // Set LED pins as outputs
  for (int i = 0; i < numInputs; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {
  for (int i = 0; i < numInputs; i++) {
    // Read analog input
    int sensorValue = analogRead(analogInputPins[i]);
    
    // Control LED based on threshold
    if (sensorValue > thresholds[i]) {
      ledStates[i] = true;
      digitalWrite(ledPins[i], HIGH);
    } else {
      ledStates[i] = false;
      digitalWrite(ledPins[i], LOW);
    }
    
    // Print values to serial for calibration
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorValue);
    Serial.print(" (Threshold: ");
    Serial.print(thresholds[i]);
    Serial.print(") | LED State: ");
    Serial.println(ledStates[i]);
  }
  
  Serial.println("--------------------");
}