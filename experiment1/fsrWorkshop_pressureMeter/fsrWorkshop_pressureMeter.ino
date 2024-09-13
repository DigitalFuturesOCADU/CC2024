/*
Creation & Computation
FSR sensor workshop - pressure meter

This code builds on the basic knightrider setup by mapping the value of a pressure sensor to the number of leds lit up

*1 Connect an FSR using a 10K resistor to pin A0
*2 Open the Serial monitor to view the raw and mapped value from the sensor
*3 Adjust the max_pressure variable to match the value range that represents your sensor

*/
// Global variables 
int sensor_pin = A0;  // Analog input pin for the pressure sensor
int first_led_pin = 2;  // Digital pin for the first LED
int min_pressure = 0; // Value with no force applied
int max_pressure = 1023;  // Maximum pressure value (adjust as needed)
const int num_leds = 6;     // Number of LEDs used for display

// Array to store LED pin numbers
int led_pins[num_leds];

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize LED pins as outputs
  for (int i = 0; i < num_leds; i++) {
    led_pins[i] = first_led_pin + i;
    pinMode(led_pins[i], OUTPUT);
  }
}

void loop() {
  // Read the analog value from the pressure sensor
  int sensor_value = analogRead(sensor_pin);
  
  // Map the sensor value to the number of LEDs to light up
  int leds_to_light = map(sensor_value, min_pressure, max_pressure, 0, num_leds);
  
  // Light up the appropriate number of LEDs
  for (int i = 0; i < num_leds; i++) {
    if (i < leds_to_light) {
      digitalWrite(led_pins[i], HIGH);  // Turn on LED
    } else {
      digitalWrite(led_pins[i], LOW);   // Turn off LED
    }
  }
  
  // Print the sensor value and number of lit LEDs for debugging
  Serial.print("Sensor value: ");
  Serial.print(sensor_value);
  Serial.print(" | LEDs lit: ");
  Serial.println(leds_to_light);
  

}