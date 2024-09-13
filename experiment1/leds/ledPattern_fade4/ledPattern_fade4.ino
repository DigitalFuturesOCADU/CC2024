/*
Control the fading rate of 4 of the LEDs
**It is only 4 LEDs, because only 4 of them are analog outputs
**You can tell because the pin is labelled with a ~

Adjust the 3 values for each LED to change the pattern


*/

int led1_Pin = 2;
int led1_minBrightness = 0;    // Minimum brightness for oscillating LED (0-255)
int led1_maxBrightness = 255;  // Maximum brightness for oscillating LED (0-255)
int led1_oscillationTime = 1000; // Time for one complete oscillation (milliseconds)

int led2_Pin = 3;
int led2_minBrightness = 255;    // Minimum brightness for oscillating LED (0-255)
int led2_maxBrightness = 0;  // Maximum brightness for oscillating LED (0-255)
int led2_oscillationTime = 900; // Time for one complete oscillation (milliseconds)

int led3_Pin = 5;
int led3_minBrightness = 100;    // Minimum brightness for oscillating LED (0-255)
int led3_maxBrightness = 255;  // Maximum brightness for oscillating LED (0-255)
int led3_oscillationTime = 800; // Time for one complete oscillation (milliseconds)

int led4_Pin = 6;
int led4_minBrightness = 100;    // Minimum brightness for oscillating LED (0-255)
int led4_maxBrightness = 10;  // Maximum brightness for oscillating LED (0-255)
int led4_oscillationTime = 700; // Time for one complete oscillation (milliseconds)


void setup() 
{
  // Initialize LED pins as outputs
  pinMode(led1_Pin, OUTPUT);
  pinMode(led2_Pin, OUTPUT);
  pinMode(led3_Pin, OUTPUT);
  pinMode(led4_Pin, OUTPUT);


}

void loop() 
{
  // Control the oscillating LED 1
  int led1_brightness = oscillate(led1_minBrightness, led1_maxBrightness, led1_oscillationTime);
  analogWrite(led1_Pin, led1_brightness);

  // Control the oscillating LED 2
  int led2_brightness = oscillate(led2_minBrightness, led2_maxBrightness, led2_oscillationTime);
  analogWrite(led2_Pin, led2_brightness);

  // Control the oscillating LED 3
  int led3_brightness = oscillate(led3_minBrightness, led3_maxBrightness, led3_oscillationTime);
  analogWrite(led3_Pin, led3_brightness);

  // Control the oscillating LED 4
  int led4_brightness = oscillate(led4_minBrightness, led4_maxBrightness, led4_oscillationTime);
  analogWrite(led4_Pin, led4_brightness);


  
}

// Function to calculate oscillating values
int oscillate(int minVal, int maxVal, int oscillationTime) {
  // Calculate current position in oscillation cycle (0.0 to 1.0)
  float cyclePosition = (millis() % oscillationTime) / (float)oscillationTime;
  
  // Calculate sine value (-1 to 1) for current cycle position
  float sineValue = sin(cyclePosition * 2 * PI);

  //Map sine wave (-1 to 1) to desired range (minVal to maxVal)
  //because the map function only returns integers, the value is multiplied by 1000
  int brightness = map(sineValue * 1000, -1000, 1000, minVal, maxVal);
  
  //output the value
  return brightness;
}
