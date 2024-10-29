/*
Multi-Servo Control via Serial
-----------------------------
This sketch reads comma-separated values over Serial and applies them as angles to servos.
- Each value should be between 0-180 (servo angles in degrees)
- The sketch automatically handles 1-4 values in the input
- Values are applied to servos in order (first value = first servo, etc.)

Hardware Setup:
--------------
1. Connect servos to the following Arduino pins:
   - Servo 1: Pin 2
   - Servo 2: Pin 3
   - Servo 3: Pin 4
   - Servo 4: Pin 5
2. Power your servos appropriately (most servos need external power)
   - DO NOT power multiple servos directly from Arduino's 5V!
   - Use an external power supply or battery pack

Usage Examples:
--------------
Send these strings over Serial (end with newline):
  "90"          -> Sets first servo to 90 degrees
  "90,180"      -> Sets first servo to 90, second to 180
  "90,180,45"   -> Sets first three servos
  "90,180,45,0" -> Sets all four servos

Serial Configuration:
-------------------
- Baud Rate: 57600
- Line Ending: Newline (\n)

Note: You can modify maxServos constant to support fewer/more servos
(remember to update servoPins array accordingly)
*/
#include <Servo.h>

// Define maximum number of servos
const int maxServos = 4;

// Create servo objects
Servo servos[maxServos];  // Array to hold servo objects

// Define constants for servo pin numbers
const int servoPins[maxServos] = {2, 3, 4, 5};  // Pins for servos 1-4

void setup() {
  // Initialize serial communication at 57600 baud rate
  Serial.begin(57600);
  
  // Attach all servos to their pins
  for(int i = 0; i < maxServos; i++) {
    servos[i].attach(servoPins[i]);
  }
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
  int lastIndex = 0;
  int servoIndex = 0;
  
  // Keep finding commas until we run out or hit our servo limit
  while(servoIndex < maxServos) {
    int nextComma = input.indexOf(',', lastIndex);
    String valueString;
    
    // Extract the value string
    if(nextComma == -1) {
      // Last value
      valueString = input.substring(lastIndex);
      // Convert to int, constrain, and apply to final servo
      servos[servoIndex].write(constrain(valueString.toInt(), 0, 180));
      break;  // No more values to process
    } else {
      // Not the last value
      valueString = input.substring(lastIndex, nextComma);
      // Convert to int, constrain, and apply to current servo
      servos[servoIndex].write(constrain(valueString.toInt(), 0, 180));
      // Move to next value
      lastIndex = nextComma + 1;
      servoIndex++;
    }
  }
}