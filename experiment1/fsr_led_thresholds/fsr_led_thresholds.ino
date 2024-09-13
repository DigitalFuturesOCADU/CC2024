/*
Experiment 1: Analog Thresholds - 5 Sensor Values

OCADU DIGF-6037 Creation & Computation - taught by Kate Hartman & Nick Puckett

For 5 sensors and 5 LEDs.
When the value of the sensor goes above its threshold value, the corresponding LED lights up.
*/

const int numInputs = 5;

// Analog input pins
const int analogInputPins[] = { A0, A1, A2, A3, A4 };
// LED output pins
const int ledPins[] = { 2, 3, 4, 5, 6 };

// Individual thresholds for each analog input
int thresholds[] = { 512, 512, 512, 512, 800 };

// Boolean variables for LED states
bool ledStates[] = { false, false, false, false, false };

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

  delay(50);  // delay in between reads for stability
}