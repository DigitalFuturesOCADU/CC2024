/*
Control the blink rate of all 6 LEDs to create a pattern
For each of the LEDs you can control
Speed of the blinking
ledXX_BlinkTime in milliseconds

Set whehter it starts as ON/OFF
ledXX_BlinkBeginState
*/

int led1_Pin = 2;
int led1_BlinkTime = 200;      // Time for one complete blink cycle (milliseconds)
bool led1_BlinkBeginState = false; // Initial state for blinking LED (true = on, false = off)

int led2_Pin = 3;
int led2_BlinkTime = 250;      // Time for one complete blink cycle (milliseconds)
bool led2_BlinkBeginState = false; // Initial state for blinking LED (true = on, false = off)

int led3_Pin = 4;
int led3_BlinkTime = 500;      // Time for one complete blink cycle (milliseconds)
bool led3_BlinkBeginState = false; // Initial state for blinking LED (true = on, false = off)

int led4_Pin = 5;
int led4_BlinkTime = 500;      // Time for one complete blink cycle (milliseconds)
bool led4_BlinkBeginState = false; // Initial state for blinking LED (true = on, false = off)

int led5_Pin = 6;
int led5_BlinkTime = 250;      // Time for one complete blink cycle (milliseconds)
bool led5_BlinkBeginState = false; // Initial state for blinking LED (true = on, false = off)

int led6_Pin = 7;
int led6_BlinkTime = 200;      // Time for one complete blink cycle (milliseconds)
bool led6_BlinkBeginState = false; // Initial state for blinking LED (true = on, false = off)

void setup() 
{
  // Initialize LED pins as outputs
  pinMode(led1_Pin, OUTPUT);
  pinMode(led2_Pin, OUTPUT);
  pinMode(led3_Pin, OUTPUT);
  pinMode(led4_Pin, OUTPUT);
  pinMode(led5_Pin, OUTPUT);
  pinMode(led6_Pin, OUTPUT);

}

void loop() 
{
  // Control the blinking LED 1
  bool ledState1 = blink(led1_BlinkTime, led1_BlinkBeginState);
  digitalWrite(led1_Pin, ledState1);
  
  // Control the blinking LED 2
  bool ledState2 = blink(led2_BlinkTime, led2_BlinkBeginState);
  digitalWrite(led2_Pin, ledState2);

  // Control the blinking LED 3
  bool ledState3 = blink(led3_BlinkTime, led3_BlinkBeginState);
  digitalWrite(led3_Pin, ledState3);

  // Control the blinking LED 4
  bool ledState4 = blink(led4_BlinkTime, led4_BlinkBeginState);
  digitalWrite(led4_Pin, ledState4);

  // Control the blinking LED 5
  bool ledState5 = blink(led5_BlinkTime, led5_BlinkBeginState);
  digitalWrite(led5_Pin, ledState5);

  // Control the blinking LED 6
  bool ledState6 = blink(led6_BlinkTime, led6_BlinkBeginState);
  digitalWrite(led6_Pin, ledState6);

}

// Function to determine LED state for blinking
//uses a sin function to control the behaviour
//blinkTime = how fast it is blinking (milliseconds)
//beginState = determines whether its starting state is ON or OFF
bool blink(int blinkTime, bool beginState) {
  // Calculate current position in blink cycle
  unsigned long currentTime = millis() % blinkTime;
  
  // Determine if we're in the first or second half of the cycle
  // This attaches the result of the if statement to the variable
  bool isFirstHalf;
  if(currentTime < (blinkTime / 2))
  {
    isFirstHalf = true;
  }
  else {
  isFirstHalf = false;
  }
 
// If beginState is true, LED is on in first half; if false, it's on in second half
  return beginState ? isFirstHalf : !isFirstHalf;
}