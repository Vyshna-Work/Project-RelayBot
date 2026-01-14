#include <Arduino.h> 

 

//------------ Motor Control Pins ------------ 

const int MOTORA1 = 6;  

const int MOTORA2 = 5; 

const int MOTORB1 = 10; 

const int MOTORB2 = 9; 

// motor A control leftwheel - motor B control rightwheel 

 

//----------- Ultra Sonic Sensor Pins -------- 

const int TRIG = 7; 

const int ECHO = 8; 

 

//-------------Motor Sensor Pins----------- 

const int MOTORLEFTSENSOR = 3;   

const int MOTORRIGHTSENSOR = 2;   

 

volatile long motorLeft_count = 0; 

volatile long motorRight_count = 0; 

 

//-------------Movement Distance------------- 

const int backwardDistance = 100; 

const int curveRight90Distance = 50; 

const int curveLeft90Distance = 300; 

 

//-------------Line Sesnors-------------- 

const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // lineSensorPins 

int thresholds[8]; // Thresholds calculated during auto-callibration 

int lastError = 0; // to remember the last turn after losing the line 

 

//------------ Function Prototypes ----------- 

void motorStop(); 

void motorForward(); 

void motorForwardSlowly(); 

void motorBackward(int distance); 

void curveRight90(int distance); 

void curveLeft90(int distance);  

void turnAround180(); 

float ultraSonicSensor(); 

void movingForwardAvoidObstacle(); 

void obstacleRoutine(); 

void motorCountleft(); 

void motorCountRight(); 

void CountReset(); 

void autoCalibrateSensors(); 

void lineFollowing(); 

bool sensorState(int i); 

int computeLineError(); 

 

void setup() { 

 

  //Serial.begin(9600); 

  // set motor pins as OUTPUT 

  pinMode(MOTORA1, OUTPUT); 

  pinMode(MOTORA2, OUTPUT); 

  pinMode(MOTORB1, OUTPUT); 

  pinMode(MOTORB2, OUTPUT); 

   

  pinMode(TRIG, OUTPUT); 

  pinMode(ECHO, INPUT); 

 

  pinMode(MOTORLEFTSENSOR, INPUT_PULLUP); 

  pinMode(MOTORRIGHTSENSOR, INPUT_PULLUP); 

 

  attachInterrupt(digitalPinToInterrupt(MOTORLEFTSENSOR),  motorCountleft, RISING); 

  attachInterrupt(digitalPinToInterrupt(MOTORRIGHTSENSOR), motorCountRight, RISING); 

 

  // set all the sensor pins as INPUT 

  for (int i = 0; i < 8; i++) { 

    pinMode(lineSensorPins[i], INPUT); 

    thresholds[i] = 514; // fixed number for black detection 

  } 

 

  // initial state 

  motorStop(); 

  //autoCalibrateSensors(); this is off for now 

} 

 

void loop() { 

  lineFollowing(); 

  //movingForwardAvoidObstacle(); 

  // motorForward(); 

  // curveRight90(); 

  // motorBackward(); 

  // curveLeft90(); 

  // turnAround180(); 

} 

//------------ Motor Control ----------------- 

void motorStop() { 

  const int stopDuration = 100; 

  unsigned long startTime = millis(); 

  while (millis() - startTime < stopDuration){ 

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, 0); 

    analogWrite(MOTORB1, 0); 

    analogWrite(MOTORB2, 0); 

  } 

} 

void motorForwardSlowly(){ 

   

  const int SPEEDMOTORA = 190; 

  const int SPEEDMOTORB = 175; 

 

  // left wheel 

  analogWrite(MOTORA1, 0); 

  analogWrite(MOTORA2, SPEEDMOTORA); 

 

  // Right wheel 

  analogWrite(MOTORB1, SPEEDMOTORB); 

  analogWrite(MOTORB2, 0); 

 

} 

void motorForward() { 

 

  const int SPEEDMOTORA = 200; 

  const int SPEEDMOTORB = 185; 

 

  // left wheel 

  analogWrite(MOTORA1, 0); 

  analogWrite(MOTORA2, SPEEDMOTORA); 

 

  // Right wheel 

  analogWrite(MOTORB1, SPEEDMOTORB); 

  analogWrite(MOTORB2, 0); 

 

} 

 

void motorBackward(int distance) { 

  CountReset(); 

  const int SPEEDMOTORA = 215; 

  const int SPEEDMOTORB = 215; 

  while (motorRight_count <= distance && motorLeft_count <= distance){ 

     

    // left wheel               

    analogWrite(MOTORA1, SPEEDMOTORA); 

    analogWrite(MOTORA2, 0); 

    // right wheel 

    analogWrite(MOTORB1, 0); 

    analogWrite(MOTORB2, SPEEDMOTORB); 

 

  } 

}  

 

void curveRight90(int distance) { 

  CountReset(); 

  const int SPEEDMOTORA = 255; 

  const int SPEEDMOTORB = 150; 

  while (motorRight_count <= distance && motorLeft_count <= distance){ 

      

    // Left wheel  

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, SPEEDMOTORA); 

    // Right wheel  

    analogWrite(MOTORB1, SPEEDMOTORB); 

    analogWrite(MOTORB2, 0); 

 

  } 

} 

 

void curveLeft90(int distance) { 

  CountReset(); 

  const int SPEEDMOTORA = 200; 

  const int SPEEDMOTORB = 255; 

  while (motorRight_count <= distance && motorLeft_count <= distance){ 

     

    // Left wheel  

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, SPEEDMOTORA); 

 

    // Right wheel 

    analogWrite(MOTORB1, SPEEDMOTORB); 

    analogWrite(MOTORB2, 0); 

  } 

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

 

  delay(1370); 

  motorStop(); 

} 

 

float ultraSonicSensor(){    // this will measure the distance to an object 

 

  float duration, distance; 

 

  digitalWrite(TRIG, LOW); 

  delayMicroseconds(2); 

  digitalWrite(TRIG, HIGH); 

  delayMicroseconds(10); 

  digitalWrite(TRIG, LOW); 

 

  duration = pulseIn(ECHO, HIGH, 30000); // 30ms timeout after that it retrun 0 

  if(duration == 0){ 

    return -1; 

  } 

  distance = (duration*.0343) / 2; // in cm 

  return distance; 

 

} 

void movingForwardAvoidObstacle() { 

  float distance = ultraSonicSensor(); 

 

  if (distance >= 9 || distance == -1) {  

   motorForward(); 

  } else { 

    obstacleRoutine(); 

  } 

} 

void obstacleRoutine() { 

  bool clear = false; 

  while(!clear){ 

    // Back up safely 

 

    motorBackward(backwardDistance); 

    // Sequence: Right → Left → Left → Right 

    curveRight90(curveRight90Distance); 

    curveLeft90(curveLeft90Distance); 

    curveLeft90(curveLeft90Distance); 

    curveRight90(curveRight90Distance); 

    clear = true; 

  } 

} 

void motorCountleft() { 

  motorLeft_count++; 

} 

 

void motorCountRight() { 

  motorRight_count++; 

} 

void CountReset(){ 

  motorLeft_count = 0; 

  motorRight_count = 0; 

} 

void autoCalibrateSensors(){ // this function: move forward slowly, detects 3 black lines and calculates the threshold 

 

  int minValue[8]; // for white backgraoud 

  int maxValue[8]; // for black line 

 

  // set min values for all sensors to 1023 

  // set max values for all sensors to 0 

  for (int i = 0; i < 8; i++){  

    minValue[i] = 1023; 

    maxValue[i] = 0; 

  } 

   

  int lineCrossing = 0; // how many blacklines crossed so far 

  bool onLine = false; // true if the robot is currently on a black line 

  motorForwardSlowly(); 

 

  // Loop only handles sensor sampling and crossing detection  

  while (lineCrossing < 10){  

    bool blackDetected = false; // no sensor see black at the start of the cycle 

     

    for (int i = 0; i < 8; i++){ // loop to read every sensor 

      int val = analogRead(lineSensorPins[i]); 

 

      // track min and max values for each sensor 

      if (val < minValue[i]){ 

        minValue[i] = val; // update min value 

      }  

      if (val > maxValue[i]){ 

        maxValue[i] = val; // update max value 

      } 

 

      // Dynamic black detection: 

      // compare current reading against the mid point of min/m ax so far 

      int dynamicThreshold = (minValue[i] + maxValue[i]) / 2; 

      if (val > dynamicThreshold){ 

        blackDetected = true; 

      } 

    } 

    // if entered black line (blackDetected = true, onLine = false) count one crossing 

    // if back to white (blackDeteced = false), reset onLine dectect next black line 

    if (blackDetected && !onLine){ 

      lineCrossing++; 

      onLine = true; 

    } else if (!blackDetected){  

      onLine = false; 

    } 

  } 

  // after the designated line crossings, compute thresholds for each sensor 

  for (int i = 0; i < 8; i++){ 

    thresholds[i] = (minValue[i] + maxValue[i]) / 2; 

  } 

} 

// check if a sensor see black or white 

bool sensorState(int i){  

  int val = analogRead(lineSensorPins[i]); 

  return val > thresholds[i]; 

} 

// compute line error using "weighted avg method" 

int computeLineError(){ 

  long weightedSum = 0; // sum of index * sensor value 

  long totalWeight = 0; // sum of sensor values 

 

  for (int i = 0; i < 8; i++){ 

    int val = analogRead(lineSensorPins[i]); 

 

    if (sensorState(i)) { 

      weightedSum += i * val; // weight index by intensity 

      totalWeight += val; // accumulate weight 

    } 

  } 

 

  if (totalWeight == 0){         // down i am fliping the negative so it turn the other way 

    return (lastError > 0 ? -5 : 5 ); // no line detected from all sensors keep turining in last known direction *these values can be tuned* 

  } 

 

  // compute weighted avarage index 

  float avgIndex = (float)weightedSum / (float)totalWeight; 

 

  // shift relative to cennter (between sensors 3 and 4) 

  int error = (int)(avgIndex - 3.5); 

 

  lastError = error; // remember last correction 

  return error; 

 

} 

void lineFollowing(){ 

  // Continuously adjust motor speeds based on line position 

  int error = computeLineError(); 

 

  const int BASE_SPEED = 180;   // base motor speed it was 150 

  const int CORRECTION = 60;    // correction factor *values can be tuned*  

 

  // Adjust speeds based on error 

  int leftSpeed  = BASE_SPEED - (error * CORRECTION);  

  int rightSpeed = BASE_SPEED + (error * CORRECTION);  

 

  // Clamp speeds to valid range 

  leftSpeed  = constrain(leftSpeed, 0, 255); 

  rightSpeed = constrain(rightSpeed, 0, 255); 

 

  // Drive motors with adjusted speeds 

  analogWrite(MOTORA1, 0); 

  analogWrite(MOTORA2, leftSpeed); 

  analogWrite(MOTORB1, rightSpeed); 

  analogWrite(MOTORB2, 0); 

 

}