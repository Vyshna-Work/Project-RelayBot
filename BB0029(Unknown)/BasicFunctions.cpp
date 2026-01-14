#include <Arduino.h>

//------------ Motor Control Pins ------------
const int MOTORA1 = 6; 
const int MOTORA2 = 5;
const int MOTORB1 = 10;
const int MOTORB2 = 9;
// motor A control leftwheel - motor B control rightwheel

//------------ Function Prototypes -----------
void motorStop();
void motorForward();
void motorBackward();
void curveRight90();
void curveLeft90();
void turnAround180();

void setup() {

  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORB2, OUTPUT);

}

void loop() {

  motorForward();
  motorBackward();
  curveRight90();
  curveLeft90();
  turnAround180();

}
//------------ Motor Control -----------------
void motorStop() {
  // strong break
  digitalWrite(MOTORA1, HIGH);
  digitalWrite(MOTORA2, HIGH);
  digitalWrite(MOTORB1, HIGH);
  digitalWrite(MOTORB2, HIGH);

  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 3000) {
  }
}

void motorForward() {

  const int SPEEDMOTORA = 215;
  const int SPEEDMOTORB = 200;

  // left wheel
  analogWrite(MOTORA1, 0);
  analogWrite(MOTORA2, SPEEDMOTORA);

  // Right wheel
  analogWrite(MOTORB1, SPEEDMOTORB);
  analogWrite(MOTORB2, 0);

  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 5000) {
  }
  motorStop();

}

void motorBackward() {
  
  const int SPEEDMOTORA = 215;
  const int SPEEDMOTORB = 215;

  // left wheel              
  analogWrite(MOTORA1, SPEEDMOTORA);
  analogWrite(MOTORA2, 0);

  // right wheel
  analogWrite(MOTORB1, 0);
  analogWrite(MOTORB2, SPEEDMOTORB);

  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 5200) {
  }
  motorStop();
}

void curveRight90() {

  const int SPEEDMOTORA = 255;
  const int SPEEDMOTORB = 150;

  // Left wheel 
  analogWrite(MOTORA1, 0);
  analogWrite(MOTORA2, SPEEDMOTORA);

  // Right wheel 
  analogWrite(MOTORB1, SPEEDMOTORB);
  analogWrite(MOTORB2, 0);

  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 2000) {
  }
  motorStop();

}
void curveLeft90() {

  const int SPEEDMOTORA = 180;
  const int SPEEDMOTORB = 255;

  // Left wheel 
  analogWrite(MOTORA1, 0);
  analogWrite(MOTORA2, SPEEDMOTORA);

  // Right wheel
  analogWrite(MOTORB1, SPEEDMOTORB);
  analogWrite(MOTORB2, 0);

  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 2500) {
  }
  motorStop();

}

void turnAround180() {

  const int SPEEDMOTORA = 220;
  const int SPEEDMOTORB = 220;

  // Left wheel forward
  analogWrite(MOTORA1, 0);
  analogWrite(MOTORA2, SPEEDMOTORA);

  // Right wheel backward
  analogWrite(MOTORB1, 0);
  analogWrite(MOTORB2, SPEEDMOTORB);

  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 1370) {
  }
  motorStop();

}