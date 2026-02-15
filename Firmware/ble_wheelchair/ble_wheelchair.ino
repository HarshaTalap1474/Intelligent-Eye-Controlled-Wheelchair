/*
 * Project: Eye-Controlled Wheelchair (Final Production Firmware)
 * Mode: BLE (Bluetooth Low Energy)
 * Features: Kickstart (Anti-Stiction), Safety Timeout, Smart LED
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- UUID CONFIGURATION (Must match Python) ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- PINS ---
const int LED_PIN = 2; 
// Left Motors
const int ENA = 13; const int IN1 = 18; const int IN2 = 19;
// Right Motors
const int IN3 = 14; const int IN4 = 27; const int ENB = 23;

// --- SETTINGS ---
int valSpeed = 180;  // Run Speed (Safe for Board, Strong enough for Floor)
const long SAFETY_TIMEOUT = 2000; // Stop if no command for 2 seconds

// --- VARIABLES ---
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long lastCommandTime = 0;
bool isMoving = false;

// --- BLUETOOTH CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(LED_PIN, HIGH); // LED ON
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(LED_PIN, LOW); // LED OFF
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // FIX: Use String for compatibility with ESP32 Core 3.x
      String rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        char cmd = rxValue[0];
        lastCommandTime = millis(); // Reset Safety Timer
        Serial.print("Received: "); Serial.println(cmd);
        processCommand(cmd);
      }
    }
    
    // --- MAIN LOGIC ---
    void processCommand(char cmd) {
      isMoving = true;
      switch (cmd) {
        case 'F': moveForward(); break;
        case 'B': moveBackward(); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        case 'S': stopMotors(); isMoving = false; break;
        
        // Speed Control
        case '0': valSpeed = 0; break;
        case '5': valSpeed = 150; break;
        case '9': valSpeed = 255; break;
      }
    }

    // --- MOTOR DRIVERS (With Kickstart) ---
    
    // Helper: 20ms Full Power Burst to break friction
    void kickstart() {
       analogWrite(ENA, 255); analogWrite(ENB, 255);
       delay(20); 
       analogWrite(ENA, valSpeed); analogWrite(ENB, valSpeed);
    }

    void moveForward() {
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      kickstart(); 
    }

    void moveBackward() {
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      kickstart();
    }

    void turnLeft() {
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      kickstart();
    }

    void turnRight() {
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      kickstart();
    }

    void stopMotors() {
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
      digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
    }
};

void setup() {
  Serial.begin(115200);
  
  // 1. Hardware Setup
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA, OUTPUT); digitalWrite(ENA, LOW);
  pinMode(ENB, OUTPUT); digitalWrite(ENB, LOW);
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  // 2. BLE Setup
  BLEDevice::init("Wheelchair_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE | 
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); 
  BLEDevice::startAdvertising();
  
  Serial.println("Waiting for Python Controller...");
}

void loop() {
  // Reconnection Handling
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      Serial.println("Restarting advertising...");
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  // Safety Timeout (Auto-Stop if Python crashes)
  if (deviceConnected && isMoving && (millis() - lastCommandTime > SAFETY_TIMEOUT)) {
      Serial.println("Safety Stop (Signal Lost)");
      // Manually stop motors
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
      digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
      isMoving = false;
  }
}