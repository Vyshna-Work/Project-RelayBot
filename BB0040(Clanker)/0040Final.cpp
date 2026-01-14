#include <Adafruit_NeoPixel.h>

#define LED_PIN 4
#define LED_COUNT 4

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ400);

// ================== ENCODER ==========================
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;

// ================== STUCK DETECTION ==================
const unsigned long STUCK_CHECK_INTERVAL = 300;
const int MIN_PULSES = 2;
unsigned long lastStuckCheck = 0;
unsigned long lastPulseSnapshot = 0;
bool motorsShouldMove = false;

// ================== MOTOR PINS =======================
const int MOTORA1 = 6;
const int MOTORA2 = 5;
const int MOTORB1 = 10;
const int MOTORB2 = 9;
int MOTORR1 = 3;

// ================== ULTRASONIC =======================
const int trigPin = 12;
const int echoPin = 13;

// ================== SERVO SCANNER ====================
const int servoPin = 8;
const int SERVO_CENTER = 1500;
const int SERVO_RIGHT  = 800;
const int SERVO_LEFT   = 2200;

// ================== LINE SENSORS =====================
const int sensorPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
const int threshold = 800;

// ================== GRIPPER ==========================
const int GRIPPER = 11;
const int GRIPPEROPEN  = 1850;
const int GRIPPERCLOSE = 1000;

// ================== SPEED SETTINGS ===================
const int Speed1 = 255;

// ================== VARIABLES ========================
float duration;
float distance;
bool lastAllBlack = false;
int blackCount = 0;

// ================== ENCODER ISR ======================
void countPulse(){
  pulseCount++;
}

// ================== SETUP ============================
void setup() {
  Serial.begin(9600);

  strip.begin();
  strip.show();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORB2, OUTPUT);

  pinMode(MOTORR1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTORR1), countPulse, RISING);

  pinMode(servoPin, OUTPUT);
  pinMode(GRIPPER, OUTPUT);

  for(int i=0;i<8;i++) pinMode(sensorPins[i], INPUT);

  randomSeed(analogRead(A0));
}

// ================== MAIN LOOP ========================
void loop() {

  int sensorValues[8];
  bool lineDetected = false;
  bool allBlack = true;

  // ---------- Read Line Sensors ----------
  for(int i=0;i<8;i++){
    sensorValues[i] = analogRead(sensorPins[i]);
    if(sensorValues[i] > threshold) lineDetected = true;
    else allBlack = false;
  }

  // ---------- BLACK LINE EVENTS ----------
  if(allBlack && !lastAllBlack){
    blackCount++;

    if(blackCount == 4){
      motorStop();
      delay(300);
      closeGripperBlocking();
      LeftTurn(450);
    }
    else if(blackCount == 5){
      motorStop();
      delay(100);
      openGripperBlocking();

      unsigned long revStart = millis();
      while(millis() - revStart < 3000){
        analogWrite(MOTORA1, Speed1);
        analogWrite(MOTORB2, Speed1);
      }
      motorStop();
    }
  }
  lastAllBlack = allBlack;

  // ================= LINE FOLLOWING =================
  if(lineDetected){
    bool leftSide = false;
    bool rightSide = false;

    for(int i=0; i<4; i++){
      if(sensorValues[i] > threshold){
        leftSide = true;
        break;
      }
    }

    for(int i=4; i<8; i++){
      if(sensorValues[i] > threshold){
        rightSide = true;
        break;
      }
    }

    if(leftSide && !rightSide){
      analogWrite(MOTORA2, Speed1/2);
      analogWrite(MOTORB1, Speed1);
      motorsShouldMove = true;
    }
    else if(rightSide && !leftSide){
      analogWrite(MOTORA2, Speed1);
      analogWrite(MOTORB1, Speed1/2);
      motorsShouldMove = true;
    }
    else{
      motorForward();
    }
  }

  // ================= MAZE MODE ======================
  else{
    distance = getDistance();

    if(distance < 12 && distance > 0){
      slowForward();
      float d1 = getDistance();
      delay(200);
      float d2 = getDistance();
      if(abs(d2 - d1) < 3) turnSequence();
    }
    else motorForward();
  }

  // ================= STUCK DETECTION =================
  if (millis() - lastStuckCheck >= STUCK_CHECK_INTERVAL) {

    noInterrupts();
    unsigned long pulsesNow = pulseCount;
    interrupts();

    unsigned long deltaPulses = pulsesNow - lastPulseSnapshot;
    lastPulseSnapshot = pulsesNow;

    if (motorsShouldMove && deltaPulses < MIN_PULSES) {
      Serial.println("STUCK DETECTED");

      motorStop();
      delay(150);

      // --- Reverse slightly before turning ---
      reverseShort(500);
      if(random(0,2) == 0) LeftTurn(200);
      else RightTurn(200);

      motorForward();
    }
    lastStuckCheck = millis();
  }

  // ================= SPEED MONITOR ===================
  if (millis() - lastTime >= 1000) {
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    Serial.print("Speed (Pulses/Sec): ");
    Serial.println(pulses);

    lastTime = millis();
  }
}

// ================== ULTRASONIC =======================
float getDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000);
  if(duration == 0) return 0;
  return (duration * 0.0343) / 2;
}

// ================== MOTORS ===========================
void motorStop(){
  motorsShouldMove = false;
  digitalWrite(MOTORA1,LOW);
  digitalWrite(MOTORA2,LOW);
  digitalWrite(MOTORB1,LOW);
  digitalWrite(MOTORB2,LOW);
  strip.fill(strip.Color(255, 0, 0));
  strip.show();
}

void motorForward(){
  motorsShouldMove = true;
  analogWrite(MOTORA2,Speed1);
  analogWrite(MOTORB1,Speed1);
  strip.fill(strip.Color(255, 255, 255));
  strip.show();
}

void slowForward(){
  motorsShouldMove = true;
  analogWrite(MOTORA2,130);
  analogWrite(MOTORB1,130);
}

// --- REVERSE FUNCTION ---
void reverseShort(unsigned long duration_ms){
  motorsShouldMove = true;
  unsigned long start = millis();
  while(millis() - start < duration_ms){
    analogWrite(MOTORA1,130);  // backward left motor
    analogWrite(MOTORB2,130);  // backward right motor
  }
  motorStop();
}

// ================== TURNING ==========================
void RightTurn(long t){
  motorsShouldMove = false;
  unsigned long s = millis();
  while(millis()-s < t){
    analogWrite(MOTORA2,255);
    analogWrite(MOTORB2,255);
  }
  motorStop();
}

void LeftTurn(long t){
  motorsShouldMove = false;
  unsigned long s = millis();
  while(millis()-s < t){
    analogWrite(MOTORA1,255);
    analogWrite(MOTORB1,255);
  }
  motorStop();
}

// ================== SCAN & TURN ======================
float scanRight(){
  motorStop(); moveServo(SERVO_RIGHT); delay(250);
  float d = getDistance(); moveServo(SERVO_CENTER); return d;
}

float scanLeft(){
  motorStop(); moveServo(SERVO_LEFT); delay(250);
  float d = getDistance(); moveServo(SERVO_CENTER); return d;
}

void turnSequence(){
  float r = scanRight();
  float l = scanLeft();
  if(r > l && r > 20) RightTurn(475);
  else if(l > r && l > 20) LeftTurn(475);
  else RightTurn(1200);
}

// ================== SERVO ============================
void moveServo(int pulse){
  unsigned long start = millis();
  while(millis() - start < 300){
    digitalWrite(servoPin,HIGH);
    delayMicroseconds(pulse);
    digitalWrite(servoPin,LOW);
    delayMicroseconds(20000 - pulse);
  }
}

// ================== GRIPPER ==========================
void closeGripperBlocking(){
  unsigned long start = millis();
  while(millis() - start < 500){
    digitalWrite(GRIPPER,HIGH);
    delayMicroseconds(GRIPPERCLOSE);
    digitalWrite(GRIPPER,LOW);
    delayMicroseconds(20000 - GRIPPERCLOSE);
  }
}

void openGripperBlocking(){
  unsigned long start = millis();
  while(millis() - start < 500){
    digitalWrite(GRIPPER,HIGH);
    delayMicroseconds(GRIPPEROPEN);
    digitalWrite(GRIPPER,LOW);
    delayMicroseconds(20000 - GRIPPEROPEN);
  }
}