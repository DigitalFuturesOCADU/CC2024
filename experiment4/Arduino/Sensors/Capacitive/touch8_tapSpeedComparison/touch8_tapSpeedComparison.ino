/**
 * TapSpeed
 * 
 * Monitors and compares tap speeds for two capacitive touch inputs over a 2-second window.
 * Uses the Adafruit CAP1188 touch sensor to detect taps and calculate speeds.
 * 
 * Key Hardware:
 * - Adafruit CAP1188 capacitive touch sensor
 * - Reset Pin: 12 (CAP1188_RESET)
 * 
 * Configuration Constants:
 * WINDOW_SIZE    - Duration of the measurement window (2000ms)
 * MAX_TAPS       - Maximum number of taps stored per window (20)
 * readInterval   - Time between sensor reads (50ms)
 * 
 * Key Variables:
 * tapTimes0[]    - Timestamp array for input 0 taps
 * tapTimes1[]    - Timestamp array for input 1 taps
 * tapCount0      - Number of taps recorded for input 0 in current window
 * tapCount1      - Number of taps recorded for input 1 in current window
 * fasterInput    - Tracks which input has higher tap speed
 *                  Values: -1 (equal), 0 (input 0 faster), 1 (input 1 faster)
 * 
*/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

#define CAP1188_RESET  12
#define WINDOW_SIZE 2000  // 2 second window in milliseconds
#define MAX_TAPS 20      // Maximum number of taps to store per window

// Create the touch sensor object with reset pin
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);

// Global variables for basic touch handling
bool touchStates[8] = {false};
bool prevTouchStates[8] = {false};
unsigned long lastReadTime = 0;
unsigned int readInterval = 50;

// Tap tracking variables for inputs 0 and 1
unsigned long tapTimes0[MAX_TAPS];
unsigned long tapTimes1[MAX_TAPS];
int tapCount0 = 0;
int tapCount1 = 0;
int fasterInput = -1;  // -1 = equal, 0 = input 0 faster, 1 = input 1 faster

// Function to update tap counts and timing for a specific input
void updateTaps(int touchIndex, unsigned long tapTimes[], int& tapCount) {
    unsigned long currentTime = millis();
    
    // Remove old taps outside the window
    while (tapCount > 0 && (currentTime - tapTimes[0]) > WINDOW_SIZE) {
        // Shift array left to remove oldest tap
        for (int i = 0; i < tapCount - 1; i++) {
            tapTimes[i] = tapTimes[i + 1];
        }
        tapCount--;
    }
    
    // Add new tap if there's room
    if (tapCount < MAX_TAPS) {
        tapTimes[tapCount] = currentTime;
        tapCount++;
    }
}

// Function to calculate taps per second
float calculateTapsPerSecond(unsigned long tapTimes[], int tapCount) {
    if (tapCount < 2) return 0.0;
    
    unsigned long currentTime = millis();
    unsigned long windowDuration = min(WINDOW_SIZE, 
                                     currentTime - tapTimes[0]);
    
    return (float)tapCount / (windowDuration / 1000.0);
}

// Function to read touch inputs and process tap speeds
void readTouchInputs() {
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= readInterval) {
        // Store previous states
        for (uint8_t i = 0; i < 8; i++) {
            prevTouchStates[i] = touchStates[i];
        }
        
        // Read all touches at once
        uint8_t touched = touchInput.touched();
        
        // Update touch states and check for new taps
        for (uint8_t i = 0; i < 8; i++) {
            touchStates[i] = (touched & (1 << i)) != 0;
            
            if (isTouchStarted(i)) {
                if (i == 0) {
                    updateTaps(0, tapTimes0, tapCount0);
                }
                else if (i == 1) {
                    updateTaps(1, tapTimes1, tapCount1);
                }
            }
        }
        
        // Calculate speeds and determine which is faster
        float speed0 = calculateTapsPerSecond(tapTimes0, tapCount0);
        float speed1 = calculateTapsPerSecond(tapTimes1, tapCount1);
        
        // Update faster input tracking
        if (abs(speed0 - speed1) < 0.01) {  // Using small epsilon for float comparison
            fasterInput = -1;  // Equal speeds
        } else if (speed0 > speed1) {
            fasterInput = 0;
        } else {
            fasterInput = 1;
        }
        
        // Print the states and speeds
        printTapSpeeds(speed0, speed1);
        
        lastReadTime = currentTime;
    }
}

// Function to check if a specific touch input just became active
bool isTouchStarted(uint8_t touchIndex) {
    return touchStates[touchIndex] && !prevTouchStates[touchIndex];
}

// Function to print tap speeds and comparison
void printTapSpeeds(float speed0, float speed1) {
    Serial.print("Input 0 speed: ");
    Serial.print(speed0, 2);
    Serial.print(" taps/sec (");
    Serial.print(tapCount0);
    Serial.println(" taps)");
    
    Serial.print("Input 1 speed: ");
    Serial.print(speed1, 2);
    Serial.print(" taps/sec (");
    Serial.print(tapCount1);
    Serial.println(" taps)");
    
    if (speed0 > 0 || speed1 > 0) {
        Serial.print("Status: ");
        switch(fasterInput) {
            case -1:
                Serial.println("Equal speeds");
                break;
            case 0:
                Serial.println("Input 0 is faster");
                break;
            case 1:
                Serial.println("Input 1 is faster");
                break;
        }
        Serial.println();
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("CAP1188 tap speed test!");

    if (!touchInput.begin()) {
        Serial.println("CAP1188 not found");
        while (1);
    }
    Serial.println("CAP1188 found!");
    
    // Initialize tap counts
    tapCount0 = 0;
    tapCount1 = 0;
    fasterInput = -1;
}

void loop() {
    readTouchInputs();
}