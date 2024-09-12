/*
Experiment 1: Analog Read Serial 5 Values

OCADU DIGF-6037 Creation & Computation - taught by Kate Hartman & Nick Puckett
Based on: https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial


*/

int sensorValue0;
int sensorValue1;
int sensorValue2;
int sensorValue3;
int sensorValue4;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {

  sensorValue0 = analogRead(A0);  // read the input on analog pin
  Serial.print(sensorValue0);     // print the value
  Serial.print(", ");             // print a comma and space

  sensorValue1 = analogRead(A1);  // read the input on analog pin
  Serial.print(sensorValue1);     // print the value
  Serial.print(", ");             // print a comma and space

  sensorValue2 = analogRead(A2);  // read the input on analog pin
  Serial.print(sensorValue2);     // print the value
  Serial.print(", ");             // print a comma and space

  sensorValue3 = analogRead(A3);  // read the input on analog pin
  Serial.print(sensorValue3);     // print the value
  Serial.print(", ");             // print a comma and space

  sensorValue4 = analogRead(A4);  // read the input on analog pin
  Serial.print(sensorValue4);     // print the value
  Serial.println();               // print a new line

  delay(50);  // delay in between reads for stability
}
