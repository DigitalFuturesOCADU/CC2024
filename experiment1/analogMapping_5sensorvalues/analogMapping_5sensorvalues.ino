/*
Experiment 1: Analog Mapping - 5 Sensor Values

OCADU DIGF-6037 Creation & Computation - taught by Kate Hartman & Nick Puckett

The value of 5 sensors is mapped to the brightness of 5 LEDs.
*/

const int numInputs = 5;

// Analog input pins
const int analogInputPins[] = { A0, A1, A2, A3, A4 };

// LED output pins
int ledPins[] = { 2, 3, 4, 5, 6 };

// sensor value variables
int mappedSensorValue = 0;
int sensorValue = 0;

void setup() {
  Serial.begin(9600);

  // Set LED pins as outputs
  for (int i = 0; i < numInputs; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {

  // Loop through these actions for each sensor + LED.
  for (int i = 0; i < numInputs; i++) {

    // Read analog input
    sensorValue = analogRead(analogInputPins[i]);

    // print
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorValue);

    // This exception is provided because pin D4 does not have analog output capabilities. This is indicated by the lack of a "~" on the pinout diagram.

    if (analogInputPins[i] == 4) {
      // map sensor value to on or off
      mappedSensorValue = map(sensorValue, 0, 1023, 0, 1);

      // set analog output accordingly
      digitalWrite(ledPins[i], mappedSensorValue);
      Serial.print(", Mapped Light Sensor Value: ");
      Serial.println(mappedSensorValue);

    } else { //if this is not pin 4
      // map sensor value to full range of analog output
      mappedSensorValue = map(sensorValue, 0, 1023, 0, 255);

      // set analog output accordingly
      analogWrite(ledPins[i], mappedSensorValue);

      //print value
      Serial.print(", Mapped Light Sensor Value: ");
      Serial.println(mappedSensorValue);
    }
  }

  Serial.println("--------------------");

  delay(50);  // delay in between reads for stability
}
