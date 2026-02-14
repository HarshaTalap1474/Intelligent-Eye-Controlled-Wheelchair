/*
 * Project: Intelligent Eye-Controlled Wheelchair (Final Firmware)
 * Hardware: Arduino Uno + L298N + HC-05
 * Features: Speed Control, Horn, Lights, Safety Timeout
 */

#include <SoftwareSerial.h>

// --- PIN DEFINITIONS ---
// Bluetooth (HC-05)
const int BT_RX = 2;  // Connect HC-05 TX to Arduino Pin 2
const int BT_TX = 3;  // Connect HC-05 RX to Arduino Pin 3

// L298N Motor Driver----
// Left Motors (OUT1 & OUT2)
const int IN1 = 6;
const int IN2 = 7;
const int ENA = 5;    // PWM Pin for Speed Control (Remove Jumper!)

// Right Motors (OUT3 & OUT4)
const int IN3 = 8;
const int IN4 = 9;
const int ENB = 10;   // PWM Pin for Speed Control (Remove Jumper!)

// Accessories
const int buzPin = 4; // Buzzer (Changed from 2 because 2 is Bluetoo----------------------------------------th)
const int ledPin = A5; // Headlight LED

// --- VARIABLES ---
SoftwareSerial BTSerial(BT_RX, BT_TX);
int valSpeed = 255;   // Default Speed (0-255)
char command = 'S';
unsigned long lastCommand----Time = 0;
const long SAFETY_TIMEOUT = 100000; // Auto-stop after 1 sec of silence

void setup() {
  Serial.begin(9600);   // USB Debugging
  BTSerial.begin(9600); // Bluetooth Communication

  // Configure Pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(buzPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize
  stopMotors();
  Serial.println("--- SYSTEM READY ---");
}

void loop() {
  // 1. Check Bluetooth--
  if (BTSerial.available() > 0) {
    command = BTSerial.read();
    lastCommandTime = millis(); // Reset Safety Timer
    Serial.println(command);    // Debug
    
    processCommand(command);
  }

  // 2. Safety Timeout (For Eye Control Safety)
  if (millis() - lastCommandTime > SAFETY_TIMEOUT) {
    if (command != 'S' && command != 'x' && command != 'X') {
      stopMotors();
      command = 'S';
    }
  }
}

void processCommand(char cmd) {
  switch (cmd) {-
    // --- MOVEMENT ---
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    
    // --- DIAGONAL / SMOOTH TURNS ---
    case 'G': forwardLeft(); break;
    case 'H': forwardRight(); break;
    case 'I': backwardLeft(); break;
    case 'J': backwardRight(); break;

    // --- ACCESSORIES ---
    case 'S': stopMotors(); break;
    case 'Y': honkHorn(); break;
    case 'X': digitalWrite(ledPin, HIGH); break; // Light ON
    case 'x': digitalWrite(ledPin, LOW); break;  // Light OFF

    // --- SPEED CONTROL (0-9) ---
    case '0': setGlobalSpeed(0); break;
    case '1': setGlobalSpeed(25); break;
    case '2': setGlobalSpeed(50); break;
    case '3': setGlobalSpeed(75); break;
    case '4': setGlobalSpeed(100); break;
    case '5': setGlobalSpeed(125); break;
    case '6': setGlobalSpeed(150); break;
    case '7': setGlobalSpeed(175); break;
    case '8': setGlobalSpeed(200); break;
    case '9': setGlobalSpeed(255); break;
  }
}

// --- DRIVER FUNCTIONS ---

void setGlobalSpeed(int speedVal) {
  valSpeed = speedVal;
  // Apply new speed immediately if moving
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
  // Spin Turn: Left Back, Right Forward
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, valSpeed);
}

void turnRight() {
  // Spin Turn: Left Forward, Right Back
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, valSpeed);
}

// --- DIAGONAL LOGIC (Tank Drive style) ---
void forwardLeft() {
  // Left Motor Slow, Right Motor Fast
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, valSpeed / 4);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, valSpeed);
}

void forwardRight() {
  // Left Motor Fast, Right Motor Slow
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, valSpeed / 4);
}

void backwardLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, valSpeed / 4);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, valSpeed);
}

void backwardRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, valSpeed / 4);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}

void honkHorn() {
  digitalWrite(buzPin, HIGH); delay(200);
  digitalWrite(buzPin, LOW);  delay(80);
  digitalWrite(buzPin, HIGH); delay(300);
  digitalWrite(buzPin, LOW);
}