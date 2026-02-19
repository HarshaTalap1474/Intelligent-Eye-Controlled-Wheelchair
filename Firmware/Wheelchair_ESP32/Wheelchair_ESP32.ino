#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURATION ---
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Python Writes Here
#define CHARACTERISTIC_UUID_TX "1c28c688-29ce-44d5-ab46-5991444903bc" // ESP32 Sends Here (NEW)

// --- PIN DEFINITIONS ---
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
#define MOVE_TIME_LONG  5000  // 5s Forward
#define MOVE_TIME_REV   2000  // 2s Reverse
#define MOVE_TIME_SHORT 800   // 0.8s Turn
#define OBSTACLE_DIST   30    // Stop if < 30cm
#define TELEMETRY_RATE  300   // Send data every 300ms (NEW)

// Variables
BLEServer* pServer = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
BLECharacteristic* pTxCharacteristic = NULL; // NEW
bool deviceConnected = false;
bool oldDeviceConnected = false;

// State Machine
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;
unsigned long lastTelemetryTime = 0; // NEW
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
    
    long duration = pulseIn(echoPin, HIGH, 30000); 
    if (duration == 0) return 999; 
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

class MyRxCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            char cmd = value[0];
            
            if (isMoving && cmd == currentAction) return;

            if (cmd == 'S') {
                stopMotors();
                return;
            }

            if (cmd == 'F') moveDuration = MOVE_TIME_LONG; 
            else if (cmd == 'B') moveDuration = MOVE_TIME_REV;  
            else if (cmd == 'L' || cmd == 'R') moveDuration = MOVE_TIME_SHORT; 
            else return;

            moveStartTime = millis();
            isMoving = true;
            currentAction = cmd;

            if (cmd == 'F') moveForward();
            else if (cmd == 'B') moveBackward();
            else if (cmd == 'L') turnLeft();
            else if (cmd == 'R') turnRight();
        }
    }
};

void setup() {
    Serial.begin(115200);
    
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
    pinMode(REAR_TRIG, OUTPUT);  pinMode(REAR_ECHO, INPUT);

    stopMotors();

    // BLE Setup
    BLEDevice::init("Wheelchair_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // 1. Create RX Characteristic (For receiving commands from Python)
    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxCharacteristic->setCallbacks(new MyRxCallbacks());

    // 2. Create TX Characteristic (For sending data to Python)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902()); // Required for Notifications

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    
    Serial.println("System Ready. Waiting for BLE connection...");
}

void loop() {
    unsigned long currentMillis = millis();

    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising(); 
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // --- TELEMETRY & COLLISION RADAR ---
    // Runs every 300ms so we don't flood the Bluetooth connection
    if (currentMillis - lastTelemetryTime >= TELEMETRY_RATE) {
        lastTelemetryTime = currentMillis;

        int fDist = getDistance(FRONT_TRIG, FRONT_ECHO);
        int rDist = getDistance(REAR_TRIG, REAR_ECHO);

        // Send Data to Python (Format: "F:45,R:120")
        if (deviceConnected) {
            String payload = "F:" + String(fDist) + ",R:" + String(rDist);
            pTxCharacteristic->setValue(payload.c_str());
            pTxCharacteristic->notify(); // Push to Python
        }

        // Active Collision Avoidance
        if (isMoving) {
            if (currentAction == 'F' && fDist < OBSTACLE_DIST) {
                Serial.println("Front Obstacle! Braking!");
                stopMotors();
            } else if (currentAction == 'B' && rDist < OBSTACLE_DIST) {
                Serial.println("Rear Obstacle! Braking!");
                stopMotors();
            }
        }
    }

    // --- PULSE TIMER ---
    if (isMoving) {
        if (currentMillis - moveStartTime >= moveDuration) {
            stopMotors();
        }
    }
}