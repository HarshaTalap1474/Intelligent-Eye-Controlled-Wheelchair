// --- NEW SAFE PIN DEFINITIONS ---
#define FRONT_TRIG 5
#define FRONT_ECHO 21

#define REAR_TRIG 32   // Moved from 4
#define REAR_ECHO 33   // Moved from 16

void setup() {
    Serial.begin(115200);
    
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(REAR_TRIG, OUTPUT);
    pinMode(REAR_ECHO, INPUT);

    Serial.println("--- Testing Safe Pins (32 & 33) ---");
    delay(1000);
}

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

void loop() {
    int frontDist = getDistance(FRONT_TRIG, FRONT_ECHO);
    int rearDist = getDistance(REAR_TRIG, REAR_ECHO);

    Serial.print("FRONT (5/21): ");
    if (frontDist == 999) Serial.print("CLEAR (>3m)   ");
    else { Serial.print(frontDist); Serial.print(" cm         "); }

    Serial.print("|   REAR (32/33): ");
    if (rearDist == 999) Serial.println("CLEAR (>3m)");
    else { Serial.print(rearDist); Serial.println(" cm"); }

    delay(500);
}