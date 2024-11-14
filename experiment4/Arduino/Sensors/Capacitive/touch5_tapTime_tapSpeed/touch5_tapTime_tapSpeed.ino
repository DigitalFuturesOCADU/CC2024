/*
 * AT42QT1070 Touch Sensor Reader with Duration and TPS Calculation
 * 
 * This code reads 1 capacitive touch input and calculates:
 * - Touch duration for each touch
 * - Taps per second over a 2-second sliding window
 * 
 * Pin Configuration:
 * Touch 1 - Digital Pin 5
 */

// Pin definition for touch input
const int TOUCH_1_PIN = 5;

// Variables for touch state and duration calculation
bool touch1;
bool previousTouch1State = false;
unsigned long touchStartTime = 0;
unsigned long touchSpeed = 0;

// Variables for TPS calculation
const int WINDOW_SIZE = 2000;  // 2 second window in milliseconds
const int MAX_TAPS = 20;       // Maximum number of taps to store
unsigned long tapTimes[MAX_TAPS];  // Circular buffer for tap timestamps
int tapIndex = 0;              // Current position in circular buffer
int totalTaps = 0;             // Total number of taps recorded

// Function to calculate current TPS
float calculateTPS() {
  unsigned long currentTime = millis();
  unsigned long windowStart = currentTime - WINDOW_SIZE;
  int tapsInWindow = 0;
  
  // Count taps within the window
  for (int i = 0; i < totalTaps; i++) {
    if (tapTimes[i] >= windowStart && tapTimes[i] <= currentTime) {
      tapsInWindow++;
    }
  }
  
  // Calculate and return TPS
  return (float)tapsInWindow / 2.0; // Divide by 2 since window is 2 seconds
}

void setup() {
  Serial.begin(9600);
  pinMode(TOUCH_1_PIN, INPUT_PULLUP);
  
  // Initialize tap times array
  for (int i = 0; i < MAX_TAPS; i++) {
    tapTimes[i] = 0;
  }
}

void loop() {
  // Read touch state
  touch1 = !digitalRead(TOUCH_1_PIN);

  // Detect touch state changes
  if (touch1 != previousTouch1State) {
    if (touch1) {
      // Touch started
      touchStartTime = millis();
      Serial.println("Touch started");
    } else {
      // Touch ended - record duration
      touchSpeed = millis() - touchStartTime;
      
      // Record tap time in circular buffer
      tapTimes[tapIndex] = millis();
      tapIndex = (tapIndex + 1) % MAX_TAPS;
      if (totalTaps < MAX_TAPS) totalTaps++;
      
      // Calculate and display metrics
      float currentTPS = calculateTPS();
      
      // Print results
      Serial.println("\n--- Touch Metrics ---");
      Serial.print("Duration: ");
      Serial.print(touchSpeed);
      Serial.println(" ms");
      Serial.print("Taps per second: ");
      Serial.println(currentTPS, 2);  // Display with 2 decimal places
      Serial.println("------------------\n");
    }
    previousTouch1State = touch1;
  }

  // Clean up old tap records
  unsigned long currentTime = millis();
  unsigned long windowStart = currentTime - WINDOW_SIZE;
  
  // Remove taps that are outside the window
  for (int i = 0; i < totalTaps; i++) {
    if (tapTimes[i] < windowStart) {
      tapTimes[i] = 0;
    }
  }


}