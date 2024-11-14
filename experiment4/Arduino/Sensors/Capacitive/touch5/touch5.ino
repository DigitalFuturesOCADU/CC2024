/*
 * AT42QT1070 Touch Sensor Reader
 * 
 * This code reads 5 capacitive touch inputs from the AT42QT1070 board
 * using direct digital pins. The sensor outputs are active LOW, so we use
 * INPUT_PULLUP and invert the readings.
 * 
 * Pin Configuration:
 * Touch 0 - Digital Pin 4
 * Touch 1 - Digital Pin 5
 * Touch 2 - Digital Pin 6
 * Touch 3 - Digital Pin 7
 * Touch 4 - Digital Pin 8
 * 
 * Variables:
 * touch0 through touch4 - Boolean values for each touch input
 *   true = touched
 *   false = not touched
 */

// Pin definitions for touch inputs
const int TOUCH_0_PIN = 4;
const int TOUCH_1_PIN = 5;
const int TOUCH_2_PIN = 6;
const int TOUCH_3_PIN = 7;
const int TOUCH_4_PIN = 8;

// Variables to store touch states
bool touch0;
bool touch1;
bool touch2;
bool touch3;
bool touch4;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure touch input pins with pullup resistors
  pinMode(TOUCH_0_PIN, INPUT_PULLUP);
  pinMode(TOUCH_1_PIN, INPUT_PULLUP);
  pinMode(TOUCH_2_PIN, INPUT_PULLUP);
  pinMode(TOUCH_3_PIN, INPUT_PULLUP);
  pinMode(TOUCH_4_PIN, INPUT_PULLUP);
}

void loop() {
  // Read and invert touch values (LOW = touched, HIGH = not touched)
  touch0 = !digitalRead(TOUCH_0_PIN);
  touch1 = !digitalRead(TOUCH_1_PIN);
  touch2 = !digitalRead(TOUCH_2_PIN);
  touch3 = !digitalRead(TOUCH_3_PIN);
  touch4 = !digitalRead(TOUCH_4_PIN);

  // Print touch states to serial monitor
  Serial.print("Touch States: ");
  Serial.print(touch0);
  Serial.print(", ");
  Serial.print(touch1);
  Serial.print(", ");
  Serial.print(touch2);
  Serial.print(", ");
  Serial.print(touch3);
  Serial.print(", ");
  Serial.println(touch4);


}