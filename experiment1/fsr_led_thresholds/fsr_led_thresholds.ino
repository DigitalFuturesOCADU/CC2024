const int numInputs = 5;

// Analog input pins
const int analogInputPins[numInputs] = {A0, A1, A2, A3, A4};

// LED output pins
const int ledPins[numInputs] = {2, 3, 4, 5, 6};

// Individual thresholds for each analog input
int threshold1 = 512;
int threshold2 = 512;
int threshold3 = 512;
int threshold4 = 512;
int threshold5 = 512;
int threshold6 = 512;

void setup() {
  Serial.begin(9600);
  
  // Set LED pins as outputs
  for (int i = 0; i < numInputs; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {
  // Read analog inputs and control LEDs
  int sensorValue1 = analogRead(analogInputPins[0]);
  int sensorValue2 = analogRead(analogInputPins[1]);
  int sensorValue3 = analogRead(analogInputPins[2]);
  int sensorValue4 = analogRead(analogInputPins[3]);
  int sensorValue5 = analogRead(analogInputPins[4]);
  int sensorValue6 = analogRead(analogInputPins[5]);
  
  // Control LEDs based on thresholds
  digitalWrite(ledPins[0], sensorValue1 > threshold1 ? HIGH : LOW);
  digitalWrite(ledPins[1], sensorValue2 > threshold2 ? HIGH : LOW);
  digitalWrite(ledPins[2], sensorValue3 > threshold3 ? HIGH : LOW);
  digitalWrite(ledPins[3], sensorValue4 > threshold4 ? HIGH : LOW);
  digitalWrite(ledPins[4], sensorValue5 > threshold5 ? HIGH : LOW);
  digitalWrite(ledPins[5], sensorValue6 > threshold6 ? HIGH : LOW);
  

  
  // Print values to serial for calibration
  Serial.print("Sensor 1: ");
  Serial.print(sensorValue1);
  Serial.print(" (Threshold: ");
  Serial.println(threshold1);

  
  Serial.print("Sensor 2: ");
  Serial.print(sensorValue2);
  Serial.print(" (Threshold: ");
  Serial.println(threshold2);

  
  Serial.print("Sensor 3: ");
  Serial.print(sensorValue3);
  Serial.print(" (Threshold: ");
  Serial.println(threshold3);

  
  Serial.print("Sensor 4: ");
  Serial.print(sensorValue4);
  Serial.print(" (Threshold: ");
  Serial.println(threshold4);

  
  Serial.print("Sensor 5: ");
  Serial.print(sensorValue5);
  Serial.print(" (Threshold: ");
  Serial.println(threshold5);


}
