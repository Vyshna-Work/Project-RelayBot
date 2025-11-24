#include <Arduino.h>
//------------ Motor Control Pins ------------
const int MOTORA1 = 6;  // Left Backwards
const int MOTORA2 = 5;  // Left Forwards
const int MOTORB1 = 10; // Right Forwards
const int MOTORB2 = 9;  // Right Backwards
// Motor A controls left wheel, motor B controls right wheel

//------------ Function Prototypes -----------
void motorStop();
void motorForward(unsigned long duration);
void motorBackward(unsigned long duration);
void LeftTurn(unsigned long duration);
void RightTurn(unsigned long duration);
void Right180turn(unsigned long duration);
void Left180turn(unsigned long duration);

void setup() {
  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORB2, OUTPUT);
}

void loop() {
  motorForward(2000);
  motorStop();
  delay(1000);

  motorBackward(2000);
  motorStop();
  delay(1000);

  LeftTurn(800);
  motorStop();
  delay(1000);

  RightTurn(800);
  motorStop();
  delay(1000);

  Right180turn(1200);
  motorStop();
  delay(1000);

  Left180turn(1200);
  motorStop();
  delay(1000);
}

void motorStop() {
  digitalWrite(MOTORA1, LOW);
  digitalWrite(MOTORA2, LOW);
  digitalWrite(MOTORB1, LOW);
  digitalWrite(MOTORB2, LOW);
}

// Move Forward
void motorForward(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(MOTORA1, LOW);
    digitalWrite(MOTORA2, HIGH);
    digitalWrite(MOTORB1, HIGH);
    digitalWrite(MOTORB2, LOW);
  }
  motorStop();
}

// Move Backward
void motorBackward(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(MOTORA1, HIGH);
    digitalWrite(MOTORA2, LOW);
    digitalWrite(MOTORB1, LOW);
    digitalWrite(MOTORB2, HIGH);
  }
  motorStop();
}

// Left Turn
void LeftTurn(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(MOTORA1, LOW);
    digitalWrite(MOTORA2, LOW);
    digitalWrite(MOTORB1, HIGH);
    digitalWrite(MOTORB2, LOW);
  }
  motorStop();
}

// Right Turn
void RightTurn(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(MOTORA1, LOW);
    digitalWrite(MOTORA2, HIGH);
    digitalWrite(MOTORB1, LOW);
    digitalWrite(MOTORB2, LOW);
  }
  motorStop();
}

// Right 180 Turn
void Right180turn(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(MOTORA1, LOW);
    digitalWrite(MOTORA2, HIGH);
    digitalWrite(MOTORB1, LOW);
    digitalWrite(MOTORB2, HIGH);
  }
  motorStop();
}

// Left 180 Turn
void Left180turn(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(MOTORA1, HIGH);
    digitalWrite(MOTORA2, LOW);
    digitalWrite(MOTORB1, HIGH);
    digitalWrite(MOTORB2, LOW);
  }
  motorStop();
}