/*
Experiment 1: Analog Mapping - 1 Sensor Value

OCADU DIGF-6037 Creation & Computation - taught by Kate Hartman & Nick Puckett

The value of the sensor is mapped to the brightness of the LED.
*/

// initialize variables
int sensorValue = 0;
int sensorPin = A0;
int LEDpin = 2;
int mappedSensorValue = 0;

void setup() {
  // initialize serial communication: 
  Serial.begin(9600);
}

void loop() {
  // read sensor pin and store value in a variable: 
  sensorValue = analogRead(sensorPin); 

  // map sensor value 
  mappedSensorValue = map(sensorValue, 0, 1023, 0, 255);

  // set analog output accordingly 
  analogWrite(LEDpin, mappedSensorValue);

  // print the sensor and mapped sensor values:
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(", Mapped Sensor Value: ");
  Serial.println(mappedSensorValue);

  // delay between readings:
  delay(100);
}