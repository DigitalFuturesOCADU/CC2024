/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myServo1;  // create servo object to control a servo
Servo myServo2;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myServo1.attach(2);  // attaches the servo on pin 9 to the servo object
  myServo2.attach(3); 
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServo1.write(pos);              // tell servo to go to position in variable 'pos'
    myServo2.write(pos); 
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myServo1.write(pos);              // tell servo to go to position in variable 'pos'
    myServo2.write(pos); 
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}
