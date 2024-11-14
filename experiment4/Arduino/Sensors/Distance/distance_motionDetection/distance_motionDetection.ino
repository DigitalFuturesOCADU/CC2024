/*******************************************************************************
 * Distance Sensor Motion Detection System
 * Tracks object motion and total distance moved over time
 *******************************************************************************/

#include <HCSR04.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 3

UltraSonicDistanceSensor distanceSensor(TRIGGER_PIN, ECHO_PIN);

// Distance sensing variables
float distance = 0.0f;
float smoothedDistance = 0.0f;
unsigned long lastDistanceReadTime = 0;
unsigned int distanceReadInterval = 100;

// Rolling average variables
const int DISTANCE_AVERAGE_WINDOW = 5;
float distanceReadings[DISTANCE_AVERAGE_WINDOW];
int distanceReadIndex = 0;
float distanceTotalValue = 0;

// Motion detection variables
const float MOTION_THRESHOLD_CM = 0.5;
const float STILL_THRESHOLD_CM = 0.2;
unsigned long motionDetectInterval = 500;
unsigned long lastMotionCheckTime = 0;
float lastSmoothedDistance = 0;
float totalMotion = 0.0f;


void setup() {
  Serial.begin(9600);
  Serial.println("Distance Motion Detector Active!");
  initializeDistanceAverage();
}

void loop() {
  readDistance();
}


void initializeDistanceAverage() {
  for (int i = 0; i < DISTANCE_AVERAGE_WINDOW; i++) {
    distanceReadings[i] = 0;
  }
  distanceTotalValue = 0;
  distanceReadIndex = 0;
}

void updateDistanceAverage(float newValue) {
  if (newValue > 0) {
    distanceTotalValue = distanceTotalValue - distanceReadings[distanceReadIndex];
    distanceReadings[distanceReadIndex] = newValue;
    distanceTotalValue = distanceTotalValue + newValue;
    distanceReadIndex = (distanceReadIndex + 1) % DISTANCE_AVERAGE_WINDOW;
    smoothedDistance = distanceTotalValue / DISTANCE_AVERAGE_WINDOW;
  }
}

void printMotionState() {
  unsigned long currentTime = millis();
  if (currentTime - lastMotionCheckTime >= motionDetectInterval) {
    float change = smoothedDistance - lastSmoothedDistance;
    
    if (abs(change) > STILL_THRESHOLD_CM) {
      totalMotion += abs(change);
    }
    
    Serial.print("Distance Raw: ");
    Serial.print(distance);
    Serial.print(" cm\tSmoothed: ");
    Serial.print(smoothedDistance);
    Serial.print(" cm\tMotion: ");
    
    if (abs(change) < STILL_THRESHOLD_CM) {
      Serial.print("STILL");
    } else if (change < -MOTION_THRESHOLD_CM) {
      Serial.print("TOWARD");
    } else if (change > MOTION_THRESHOLD_CM) {
      Serial.print("AWAY");
    }
    
    Serial.print("\tTotal Motion: ");
    Serial.println(totalMotion);
    
    lastSmoothedDistance = smoothedDistance;
    lastMotionCheckTime = currentTime;
  }
}

void readDistance() {
  unsigned long currentTime = millis();
  if (currentTime - lastDistanceReadTime >= distanceReadInterval) {
    float newDistance = distanceSensor.measureDistanceCm();
    
    if (newDistance > 0) {
      distance = newDistance;
      updateDistanceAverage(distance);
    }
    
    printMotionState();
    lastDistanceReadTime = currentTime;
  }
}
