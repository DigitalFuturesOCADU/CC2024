/*
 * AT42QT1070 Touch Sensor Reader with Digital Buzzer Control
 * 
 * Reads 2 capacitive touch inputs and controls a piezo buzzer
 * using digital on/off at specified intervals.
 * 
 * Pin Configuration:
 * Touch 0 - Digital Pin 4  
 * Touch 1 - Digital Pin 5  
 * Buzzer  - Digital Pin 11
 *
 * Interaction
 * Touch 0 - State 1 - BEEP
 * Touch 1 - State 2 - BOOP
 * None    - State 0 - :(
 */

// Pin definitions
const int TOUCH_0_PIN = 4;
const int TOUCH_1_PIN = 5;
const int BUZZER_PIN = 11;

// Variables to store touch states
bool touch0;
bool touch1;
int inputState = 0;

// Timing variables
unsigned long previousMillis = 0;
bool buzzerState = false;  // Track if buzzer is currently on or off

void setup() {
  Serial.begin(9600);
  
  // Configure touch input pins with pullup resistors
  pinMode(TOUCH_0_PIN, INPUT_PULLUP);
  pinMode(TOUCH_1_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // Read and invert touch values (LOW = touched, HIGH = not touched)
  touch0 = !digitalRead(TOUCH_0_PIN);
  touch1 = !digitalRead(TOUCH_1_PIN);

  // Determine input state
  if (touch0) {
    inputState = 1;
  } else if (touch1) {
    inputState = 2;
  } else {
    inputState = 0;
  }

  // Call buzzer output function
  buzzerOutput(inputState);


}

void buzzerOutput(int state) {
  unsigned long currentMillis = millis();
  int interval;
  
  
  switch(state) {
    case 1:  // Touch 0 active
      interval = 3;  // 3ms interval
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        buzzerState = !buzzerState;  // Toggle buzzer state
        digitalWrite(BUZZER_PIN, buzzerState);
        if(buzzerState) {  // Only print on rising edge
          Serial.print(state);
          
          Serial.println(" : BEEP");
        }
      }
      break;
      
    case 2:  // Touch 1 active
      interval = 20;  // 20ms interval
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        buzzerState = !buzzerState;  // Toggle buzzer state
        digitalWrite(BUZZER_PIN, buzzerState);
        if(buzzerState) {  // Only print on rising edge
          Serial.print(state);
          
          Serial.println(" : BOOP");
        }
      }
      break;
      
    default:  // No touch (state 0)
      digitalWrite(BUZZER_PIN, LOW);  // Turn off buzzer
      buzzerState = false;  // Reset buzzer state
      Serial.print(state);
      Serial.println(" :(");
      break;
  }
}