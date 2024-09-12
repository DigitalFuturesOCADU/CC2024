/*
Experiment 1: Analog Mapping - 1 Sensor Value

OCADU DIGF-6037 Creation & Computation - taught by Kate Hartman & Nick Puckett

The value of the sensor is mapped to the brightness of the LED.
*/

// initialize variables
int sensorValue = 0;
int sensorPin = A0;
int LEDpin = 2;
int threshold = 500;  // change this number if a higher or lower threshold is needed

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
}

void loop() {
  // read light sensor pin and store value in a variable:
  sensorValue = analogRead(sensorPin);

  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(" (Threshold: ");
  Serial.print(threshold);
  Serial.print("), LED: ");

  if (sensorValue > threshold){
      // set analog output accordingly
      digitalWrite(LEDpin, HIGH);
      Serial.println("on!");
    }
  else {
    // set analog output accordingly
    digitalWrite(LEDpin, LOW);
    Serial.println("off");
  }

  // delay between readings:
  delay(50);
}