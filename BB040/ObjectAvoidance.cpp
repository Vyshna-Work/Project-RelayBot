#include <Arduino.h>
const int MOTORA1 = 6;  // Left Backwards
const int MOTORA2 = 5;  // Left Forwards
const int MOTORB1 = 10; // Right Forwards
const int MOTORB2 = 9;  // Right Backwards
const int trigPin = 12;
const int echoPin = 13;

float duration;
float distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORB2, OUTPUT);
// Put the states the way you want it to be (LOW in this case)
  Serial.begin(9600);
}

void loop() {

  distance = getDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 25 && distance > 0) { 
    
    slowForward();
    float distance1 = getDistance();
    delay(250);
    float distance2 = getDistance();
    float change = abs(distance2 - distance1);

    Serial.print("Distance Change: ");
    Serial.println(change);

    if (change < 3) {
      avoidObstacle();
    } else {
      slowForward();
    }

  } else {

    motorForward();
  }

  delay(40);
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;
}

void motorStop() {
  digitalWrite(MOTORA1, LOW);
  digitalWrite(MOTORA2, LOW);
  digitalWrite(MOTORB1, LOW);
  digitalWrite(MOTORB2, LOW);
}

void motorForward() {
  analogWrite(MOTORA2, 255);
  analogWrite(MOTORB1, 255);
  digitalWrite(MOTORA1, LOW);
  digitalWrite(MOTORB2, LOW);
}

void slowForward() {
  analogWrite(MOTORA2, 130);
  analogWrite(MOTORB1, 130);
  digitalWrite(MOTORA1, LOW);
  digitalWrite(MOTORB2, LOW);
}

void RightTurn(long duration) {
  unsigned long startTime = millis();
  motorStop();
  while (millis() - startTime < duration) {
    analogWrite(MOTORA1, 0);
    analogWrite(MOTORA2, 255);
    analogWrite(MOTORB1, 150);
    analogWrite(MOTORB2, 0);
  }
}

void LeftTurn(long duration) {
  unsigned long startTime = millis();
  motorStop();
  while (millis() - startTime < duration) {
    analogWrite(MOTORA1, 0);
    analogWrite(MOTORA2, 200);
    analogWrite(MOTORB1, 255);
    analogWrite(MOTORB2, 0);
  }
}

void avoidObstacle() {
  RightTurn(1500);
  LeftTurn(1500);
  LeftTurn(1000);
  RightTurn(1500);
  motorForward();
}
