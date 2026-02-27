#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURATION ---
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8" 
#define CHARACTERISTIC_UUID_TX "1c28c688-29ce-44d5-ab46-5991444903bc" 

// --- PIN DEFINITIONS ---
#define ENA 13
#define IN1 18
#define IN2 19
#define IN3 14
#define IN4 27
#define ENB 23
#define LED_PIN 2

#define FRONT_TRIG 5
#define FRONT_ECHO 21
#define REAR_TRIG 32
#define REAR_ECHO 33

// --- TIMING, DISTANCE & SPEED ---
#define MOVE_TIME_LONG  5000  // 5s Forward
#define MOVE_TIME_REV   2000  // 2s Reverse
#define MOVE_TIME_SHORT 800   // 0.8s Turn
#define OBSTACLE_DIST   30    // Stop if < 30cm
#define TELEMETRY_RATE  300   // Send data every 300ms
#define RADAR_POLL_RATE 50    // Scan sensors every 50ms

#define MAX_PWM_FWD  140      // Forward top speed (0-255)
#define MAX_PWM_TURN 120      // Turning top speed (0-255)

// --- GLOBALS ---
BLEServer* pServer = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
BLECharacteristic* pTxCharacteristic = NULL; 
bool deviceConnected = false;
bool oldDeviceConnected = false;

unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;
unsigned long lastTelemetryTime = 0; 
unsigned long lastRadarTime = 0;

bool isMoving = false;
char currentAction = 'S';
int currentFrontDist = 999;
int currentRearDist = 999;

// --- MOTOR & POWER CONTROL ---
void applyPowerSoftly(int targetPWM) {
    for (int duty = 0; duty <= targetPWM; duty += 20) {
        analogWrite(ENA, duty);
        analogWrite(ENB, duty);
        delay(10); 
    }
    analogWrite(ENA, targetPWM);
    analogWrite(ENB, targetPWM);
}

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
    applyPowerSoftly(MAX_PWM_FWD);
}

void moveBackward() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    applyPowerSoftly(MAX_PWM_FWD);
}

void turnLeft() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    applyPowerSoftly(MAX_PWM_TURN);
}

void turnRight() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    applyPowerSoftly(MAX_PWM_TURN);
}

// --- SENSOR LOGIC ---
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

// --- BLUETOOTH CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        digitalWrite(LED_PIN, HIGH);
    };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        digitalWrite(LED_PIN, LOW);
        stopMotors();
    }
};

class MyRxCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            char cmd = value[0];
            
            if (cmd == 'S') { stopMotors(); return; }
            if (isMoving) return; // Command Lockout

            // Safety Pre-Check
            if (cmd == 'F' && currentFrontDist < OBSTACLE_DIST) return;
            if (cmd == 'B' && currentRearDist < OBSTACLE_DIST) return;

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

    BLEDevice::init("Wheelchair_BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyRxCallbacks());

    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902()); 

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}

void loop() {
    unsigned long currentMillis = millis();

    if (!deviceConnected && oldDeviceConnected) {
        delay(500); pServer->startAdvertising(); oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) oldDeviceConnected = deviceConnected;

    // 1. HIGH-SPEED SAFETY RADAR (50ms)
    if (currentMillis - lastRadarTime >= RADAR_POLL_RATE) {
        lastRadarTime = currentMillis;
        currentFrontDist = getDistance(FRONT_TRIG, FRONT_ECHO);
        currentRearDist = getDistance(REAR_TRIG, REAR_ECHO);

        if (isMoving) {
            if (currentAction == 'F' && currentFrontDist < OBSTACLE_DIST) stopMotors();
            else if (currentAction == 'B' && currentRearDist < OBSTACLE_DIST) stopMotors();
        }
    }

    // 2. BLUETOOTH TELEMETRY PUSH (300ms)
    if (currentMillis - lastTelemetryTime >= TELEMETRY_RATE) {
        lastTelemetryTime = currentMillis;
        if (deviceConnected) {
            String payload = "F:" + String(currentFrontDist) + ",R:" + String(currentRearDist);
            pTxCharacteristic->setValue(payload.c_str());
            pTxCharacteristic->notify(); 
        }
    }

    // 3. PULSE TIMER
    if (isMoving && (currentMillis - moveStartTime >= moveDuration)) {
        stopMotors();
    }
}