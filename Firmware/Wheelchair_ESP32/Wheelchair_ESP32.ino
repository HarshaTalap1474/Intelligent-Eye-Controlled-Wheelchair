#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURATION ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- PIN DEFINITIONS ---
// Motor Pins
#define ENA 13
#define IN1 18
#define IN2 19
#define IN3 14
#define IN4 27
#define ENB 23
#define LED_PIN 2

// VERIFIED Ultrasonic Sensor Pins
#define FRONT_TRIG 5
#define FRONT_ECHO 21
#define REAR_TRIG 32
#define REAR_ECHO 33

// --- TIMING & DISTANCE CONFIGURATION ---
#define MOVE_TIME_LONG  5000  // 5 Seconds for Forward
#define MOVE_TIME_REV   2000  // 2 Seconds for Reverse
#define MOVE_TIME_SHORT 800   // 0.8 Seconds for Left/Right
#define OBSTACLE_DIST   30    // Stop if closer than 30cm

// Variables
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// State Machine
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;
bool isMoving = false;
char currentAction = 'S';

// --- MOTOR CONTROL ---
void stopMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0); analogWrite(ENB, 0);
    isMoving = false;
    currentAction = 'S';
}

void moveForward() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, 200); analogWrite(ENB, 200);
}

void moveBackward() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 200); analogWrite(ENB, 200);
}

void turnLeft() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, 180); analogWrite(ENB, 180);
}

void turnRight() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, 180); analogWrite(ENB, 180);
}

// --- ULTRASONIC SENSOR LOGIC ---
int getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // 30ms timeout prevents freezing
    long duration = pulseIn(echoPin, HIGH, 30000); 
    if (duration == 0) return 999; // No echo = clear path
    return duration * 0.034 / 2;
}

// --- CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        digitalWrite(LED_PIN, HIGH);
        Serial.println("BLE Connected!");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        digitalWrite(LED_PIN, LOW);
        stopMotors();
        Serial.println("BLE Disconnected!");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            char cmd = value[0];
            
            // Ignore if already executing this command
            if (isMoving && cmd == currentAction) return;

            // EMERGENCY STOP
            if (cmd == 'S') {
                Serial.println("CMD: EMERGENCY STOP");
                stopMotors();
                return;
            }

            // --- SMART START LOGIC (Obstacle Pre-Check) ---
            if (cmd == 'F') {
                if (getDistance(FRONT_TRIG, FRONT_ECHO) < OBSTACLE_DIST) {
                    Serial.println("CMD: F - REJECTED (Front Blocked)");
                    return;
                }
                moveDuration = MOVE_TIME_LONG; 
                
            } else if (cmd == 'B') {
                if (getDistance(REAR_TRIG, REAR_ECHO) < OBSTACLE_DIST) {
                    Serial.println("CMD: B - REJECTED (Rear Blocked)");
                    return;
                }
                moveDuration = MOVE_TIME_REV;  
                
            } else if (cmd == 'L' || cmd == 'R') {
                moveDuration = MOVE_TIME_SHORT; 
            } else {
                return;
            }

            // --- EXECUTE MOVE ---
            moveStartTime = millis();
            isMoving = true;
            currentAction = cmd;

            if (cmd == 'F') moveForward();
            else if (cmd == 'B') moveBackward();
            else if (cmd == 'L') turnLeft();
            else if (cmd == 'R') turnRight();
            
            Serial.print("CMD Executed: "); Serial.println(cmd);
        }
    }
};

void setup() {
    Serial.begin(115200);
    
    // Motor Pins
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // Sensor Pins
    pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
    pinMode(REAR_TRIG, OUTPUT);  pinMode(REAR_ECHO, INPUT);

    stopMotors();

    // BLE Setup
    BLEDevice::init("Wheelchair_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    
    Serial.println("System Ready. Waiting for BLE connection...");
}

void loop() {
    // 1. Connection Recovery
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising(); 
        oldDeviceConnected = deviceConnected;
        Serial.println("Advertising Restarted.");
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // 2. ACTIVE MOVEMENT MONITORING
    if (isMoving) {
        unsigned long elapsed = millis() - moveStartTime;

        // A. Collision Detection while moving
        if (currentAction == 'F') {
            if (getDistance(FRONT_TRIG, FRONT_ECHO) < OBSTACLE_DIST) {
                Serial.println("ALERT: Front Obstacle! Braking!");
                stopMotors();
                return;
            }
        } else if (currentAction == 'B') {
            if (getDistance(REAR_TRIG, REAR_ECHO) < OBSTACLE_DIST) {
                Serial.println("ALERT: Rear Obstacle! Braking!");
                stopMotors();
                return;
            }
        }

        // B. Timer check
        if (elapsed >= moveDuration) {
            Serial.println("Pulse Complete - Coasting to Stop.");
            stopMotors();
        }
    }
}