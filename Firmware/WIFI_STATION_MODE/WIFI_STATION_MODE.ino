/*
 * Project: Eye-Controlled Wheelchair (Final Long-Range Version)
 * Mode: STATION (Connects to Router/Hotspot for better range)
 * Features: Safe Boot (No Jerk), Smart LED, Timeout
 */

#include <WiFi.h>

// --- USER CONFIGURATION ---
const char* ssid = "HospitalWiFi";      // CHANGE THIS (Hotspot Name)
const char* password = "12345678"; // CHANGE THIS (Hotspot Pass)
const int SERVER_PORT = 80;

WiFiServer server(SERVER_PORT);

// --- PINS ---
const int LED_PIN = 2; // Status LED

// Left Motors
const int ENA = 13; const int IN1 = 18; const int IN2 = 19;
// Right Motors
const int IN3 = 14; const int IN4 = 27; const int ENB = 23;

// --- VARIABLES ---
int valSpeed = 255;
unsigned long lastCommandTime = 0;
const long SAFETY_TIMEOUT = 2000; // 2 seconds timeout
bool isMoving = false;

void setup() {
  Serial.begin(115200);

  // 1. SAFETY FIRST: KILL ALL MOTORS IMMEDIATELY
  // We configure output mode and write LOW instantly to prevent the "Jerk"
  pinMode(ENA, OUTPUT); digitalWrite(ENA, LOW);
  pinMode(ENB, OUTPUT); digitalWrite(ENB, LOW);
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
  
  pinMode(LED_PIN, OUTPUT);

  // 2. Connect to Wi-Fi (Station Mode)
  Serial.println("\n--- STARTING LONG RANGE MODE ---");
  Serial.print("Connecting to: "); Serial.println(ssid);
  
  WiFi.mode(WIFI_STA); // Station Mode (Client)
  WiFi.begin(ssid, password);

  // Blink LED while connecting
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
  }

  // 3. Connected!
  Serial.println("\n✅ Wi-Fi Connected!");
  Serial.print("ROBOT IP ADDRESS: ");
  Serial.println(WiFi.localIP()); // <--- WRITE THIS DOWN FROM SERIAL MONITOR
  
  // LED Solid ON = Ready
  digitalWrite(LED_PIN, HIGH);
  
  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Controller Connected");
    
    while (client.connected()) {
      if (client.available()) {
        char command = client.read();
        lastCommandTime = millis();
        
        // Visual Feedback: Quick Blink on Command
        digitalWrite(LED_PIN, LOW); delay(10); digitalWrite(LED_PIN, HIGH);
        
        Serial.print("Cmd: "); Serial.println(command);
        processCommand(command);
      }
      
      // Safety Timeout
      if (millis() - lastCommandTime > SAFETY_TIMEOUT && isMoving) {
        stopMotors();
        isMoving = false;
        digitalWrite(LED_PIN, LOW); // LED OFF = Safety Trip
        Serial.println("Safety Stop (Timeout)");
      }
    }
    stopMotors();
    Serial.println("Controller Disconnected");
    digitalWrite(LED_PIN, HIGH); // Back to Ready State
  }
}

// --- LOGIC ---
void processCommand(char cmd) {
  isMoving = true;
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); isMoving = false; break;
    
    // Speed Control
    case '0': setSpeed(0); break;
    case '5': setSpeed(150); break;
    case '9': setSpeed(255); break;
  }
}

// --- DRIVERS ---
void setSpeed(int s) {
  valSpeed = s;
  analogWrite(ENA, valSpeed); analogWrite(ENB, valSpeed);
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
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, valSpeed);
}
void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, valSpeed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, valSpeed);
}
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}