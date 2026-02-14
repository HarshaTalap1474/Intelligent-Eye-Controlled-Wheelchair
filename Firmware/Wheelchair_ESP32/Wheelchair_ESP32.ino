/*
 * Project: Eye-Controlled Wheelchair (ESP32 Upgrade)
 * Author: Harshavardhan Talap
 * Features: Bluetooth Serial, Safety Timeout, Speed Control
 */

#include "BluetoothSerial.h"

// Check if Bluetooth is properly enabled in settings
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// --- PIN MAPPING (Safe GPIOs) ---
// Left Motors
const int ENA = 13;
const int IN1 = 18;
const int IN2 = 19;
// Right Motors
const int IN3 = 14;
const int IN4 = 27;
const int ENB = 23;

// --- VARIABLES ---
int valSpeed = 255; // Max speed (0-255)
char command = 'S';
unsigned long lastCommandTime = 0;
const long SAFETY_TIMEOUT = 1000; // Stop after 1 second of silence

void setup() {
  Serial.begin(115200); // USB Debug
  
  // 1. Start Bluetooth with a specific name
  SerialBT.begin("Wheelchair_ESP32"); 
  Serial.println("Bluetooth Started! Pair with 'Wheelchair_ESP32'");

  // 2. Configure Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  stopMotors();
}

void loop() {
  // A. Receive Command from PC/Python
  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCommandTime = millis();
    Serial.print("Received: "); Serial.println(command);
    processCommand(command);
  }

  // B. Safety Timeout (Crucial for Eye Tracking)
  // If Python crashes or eye closes, robot MUST stop.
  if (millis() - lastCommandTime > SAFETY_TIMEOUT) {
    if (command != 'S') {
      stopMotors();
      command = 'S';
      Serial.println("Safety Stop: Signal Lost");
    }
  }
}

void processCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    
    // Speed Control (0-9)
    case '0': setSpeed(0); break;
    case '1': setSpeed(50); break;
    case '3': setSpeed(100); break;
    case '5': setSpeed(150); break;
    case '7': setSpeed(200); break;
    case '9': setSpeed(255); break;
  }
}

// --- DRIVER FUNCTIONS ---
void setSpeed(int s) {
  valSpeed = s;
  analogWrite(ENA, valSpeed);
  analogWrite(ENB, valSpeed);
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, valSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, valSpeed);
}

void turnLeft() {
  // Differential Spin
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, valSpeed);
}

void turnRight() {
  // Differential Spin
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, valSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}