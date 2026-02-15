/*
 * Project: Wheelchair Hardware Self-Test
 * Purpose: Verify wiring and motor power WITHOUT Bluetooth.
 * Sequence: Forward -> Stop -> Backward -> Stop -> Left -> Stop -> Right -> Stop
 */

// --- PINS (Matches your BLE Setup) ---
// Left Motors
const int ENA = 13; 
const int IN1 = 18; 
const int IN2 = 19;

// Right Motors
const int IN3 = 14; 
const int IN4 = 27; 
const int ENB = 23;

const int LED_PIN = 2; // Status LED

// --- SPEED SETTINGS ---
int testSpeed = 180; // Standard Speed (0-255)

void setup() {
  Serial.begin(115200);
  
  // Configure Pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Safe Start
  stopMotors();
  Serial.println("--- HARDWARE TEST STARTING IN 3 SECONDS ---");
  Serial.println("PLEASE LIFT ROBOT OFF THE GROUND");
  
  // Blink LED to warn user
  for(int i=0; i<3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(500);
    digitalWrite(LED_PIN, LOW); delay(500);
  }
}

void loop() {
  // 1. FORWARD TEST
  Serial.println("TEST: FORWARD (Both Sides)");
  digitalWrite(LED_PIN, HIGH);
  moveForward();
  delay(2000); // Run for 2 seconds
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  delay(1000); // Rest for 1 second

  // 2. BACKWARD TEST
  Serial.println("TEST: BACKWARD (Both Sides)");
  digitalWrite(LED_PIN, HIGH);
  moveBackward();
  delay(2000);
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  delay(1000);

  // 3. LEFT TURN TEST (Spin Left)
  Serial.println("TEST: LEFT TURN (Left Back, Right Fwd)");
  digitalWrite(LED_PIN, HIGH);
  turnLeft();
  delay(2000);
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  delay(1000);

  // 4. RIGHT TURN TEST (Spin Right)
  Serial.println("TEST: RIGHT TURN (Left Fwd, Right Back)");
  digitalWrite(LED_PIN, HIGH);
  turnRight();
  delay(2000);
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  delay(2000); // Long rest before restarting loop
}

// --- MOTION DRIVERS WITH KICKSTART ---

void kickstart() {
   // Brief burst of 100% power to break stiction
   analogWrite(ENA, 255); 
   analogWrite(ENB, 255);
   delay(20); 
   // Drop to test speed
   analogWrite(ENA, testSpeed); 
   analogWrite(ENB, testSpeed);
}

void moveForward() {
  // Left Fwd, Right Fwd
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  kickstart(); 
}

void moveBackward() {
  // Left Back, Right Back
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  kickstart();
}

void turnLeft() {
  // Left Back, Right Fwd
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  kickstart();
}

void turnRight() {
  // Left Fwd, Right Back
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  kickstart();
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}