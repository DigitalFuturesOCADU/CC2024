/*
Read a comma separated string over Serial and apply it as the brightness value of LEDs
*each value controls the brightness of 2 LEDs
Specifically designed for the "KnightRider" setup, but can be adapted

The sketch can be controlled from the website at this address : https://editor.p5js.org/npuckett/collections/T5352XtXk

*/



// Define constants for LED pin numbers
int led1Pin1 = 2;  // First pin for LED 1
int led1Pin2 = 3;  // Second pin for LED 1
int led2Pin1 = 5;  // First pin for LED 2
int led2Pin2 = 6;  // Second pin for LED 2

void setup() {
  // Initialize serial communication at 57600 baud rate
  Serial.begin(57600);
  
  // Set all LED pins as OUTPUT
  pinMode(led1Pin1, OUTPUT);
  pinMode(led1Pin2, OUTPUT);
  pinMode(led2Pin1, OUTPUT);
  pinMode(led2Pin2, OUTPUT);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming data until a newline character
    String input = Serial.readStringUntil('\n');
    // If the input is not empty, process it
    if (input.length() > 0) {
      processInput(input);
    }
  }
}

void processInput(String input) {
  // Find the position of the comma in the input string
  int commaIndex = input.indexOf(',');
  
  // If a comma is found in the input
  if (commaIndex != -1) {
    // Split the input string into two parts
    String value1String = input.substring(0, commaIndex);
    String value2String = input.substring(commaIndex + 1);
    
    // Convert the string values to integers and constrain them between 0 and 255
    int value1 = constrain(value1String.toInt(), 0, 255);
    int value2 = constrain(value2String.toInt(), 0, 255);
    
    // Set the brightness of LED 1 (both pins) using value1
    analogWrite(led1Pin1, value1);
    analogWrite(led1Pin2, value1);
    
    // Set the brightness of LED 2 (both pins) using value2
    analogWrite(led2Pin1, value2);
    analogWrite(led2Pin2, value2);
  }
}