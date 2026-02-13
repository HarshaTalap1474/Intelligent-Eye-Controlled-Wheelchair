#include <SoftwareSerial.h>

// Define Bluetooth Pins (RX, TX)
// Connect HC-05 TX to Arduino Pin 2
// Connect HC-05 RX to Arduino Pin 3
SoftwareSerial BTSerial(2, 3); 

char incomingByte;

void setup() {
  // Start the Serial Monitor (Communication with PC)
  Serial.begin(9600);
  Serial.println("--- DEVELOPMENT MODE STARTED ---");
  Serial.println("Waiting for Bluetooth commands...");

  // Start the Bluetooth Module (Communication with Phone/Remote)
  // Note: Default for HC-05 is usually 9600 or 38400. Try 9600 first.
  BTSerial.begin(9600); 
}

void loop() {
  // 1. Check if data is coming from Bluetooth
  if (BTSerial.available()) {
    incomingByte = BTSerial.read(); // Read the character

    // 2. Print it to the Serial Monitor for debugging
    Serial.print("Received Command: ");
    Serial.print(incomingByte);

    // 3. Decode the command (Simulate Motor Action)
    if (incomingByte == 'F') {
      Serial.println(" -> ACTION: Move Forward");
    } 
    else if (incomingByte == 'B') {
      Serial.println(" -> ACTION: Move Backward");
    } 
    else if (incomingByte == 'L') {
      Serial.println(" -> ACTION: Turn Left");
    } 
    else if (incomingByte == 'R') {
      Serial.println(" -> ACTION: Turn Right");
    } 
    else if (incomingByte == 'S') {
      Serial.println(" -> ACTION: STOP");
    }
    else {
      Serial.println(" -> [Unknown Command]");
    }
  }
}