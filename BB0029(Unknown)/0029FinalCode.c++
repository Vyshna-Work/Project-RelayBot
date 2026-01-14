#include <Adafruit_NeoPixel.h> // NeoPixel library 

 

//------------ NeoPixel Setup ---------------- 

#define NEOPIXEL_PIN 4 // connected to D4 

#define NUM_PIXELS 4 // 4 NeoPixels 

unsigned long lastBlinkTime = 0; 

bool blinkState = false; 

 

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); 

 

//------------Bot Name------------------------ 

String botName = "Unknown"; 

 

//------------ Motor Control Pins ------------ 

const int MOTORA1 = 6;  

const int MOTORA2 = 5; 

const int MOTORB1 = 10; 

const int MOTORB2 = 9; 

// motor A control leftwheel - motor B control rightwheel 

 

//----------- Ultra Sonic Sensor Pins -------- 

const int TRIG = 7; 

const int ECHO = 8; 

 

//------------ Control the Gripper-------------- 

const int GRIPPER = 12; 

const int GRIPPEROPEN = 2000; 

const int GRIPPERCLOSE = 1225; 

 

//------------ MotorSpeed--------------------- 

int leftSpeed = 0; 

int rightSpeed = 0; 

 

//------------Button --------------- 

const int BUTTON1 = 11; 

int buttonState = 0; 

//-------------Motor Sensor Pins----------- 

const int MOTORLEFTSENSOR = 3;   

const int MOTORRIGHTSENSOR = 2;   

 

volatile long motorLeft_count = 0; 

volatile long motorRight_count = 0; 

 

//-------------Movement Distance------------- 

const int backwardDistance = 90; 

const int curveRight90Distance = 50; 

const int curveLeft90Distance = 180; 

const int startingDistance = 1; 

const int curveLeftAroundItSelfDistance = 40; 

 

// --- Obstacle persistence counter --- 

static int obstacleCounter = 0;   // counts consecutive "too close" readings 

const int OBSTACLE_THRESHOLD = 15; // cm 

const int OBSTACLE_CONFIRM = 3;    // how many cycles to confirm obstacle 

const int STARTING_FLAG = 15; 

 

//-------------Line Sesnors-------------- 

const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // lineSensorPins 

int thresholds[8];  

int lastError = 0; // to remember the last turn after losing the line 

 

//------------Define the bot behavior state------------------ 

enum BotState {  // PascalCase naming for the type 

  STARTING_POINT, // UPPER_SNAKE_CASE for the values 

  SEARCH_OBJECT,  

  TURN_LEFT, 

  FOLLOW_LINE, 

  DROP_OBJECT, 

  DONE  

}; 

BotState currentState = STARTING_POINT; // start with searching for object 

 

// covert enum to text for json  

const char* stateToText(BotState state) { 

  switch (state) { 

    case STARTING_POINT:return "STARTING_POINT"; 

    case SEARCH_OBJECT: return "SEARCH_OBJECT"; 

    case TURN_LEFT:     return "TURN_LEFT"; 

    case FOLLOW_LINE:   return "FOLLOW_LINE"; 

    case DROP_OBJECT:   return "DROP_OBJECT"; 

    case DONE:          return "DONE"; 

    default:            return "UNKNOWN"; // for debugging 

  } 

} 

 

//------------ Function Prototypes ----------- 

void motorStop(); 

void motorStrongStop(); 

void motorForward(int distance); 

void motorForwardSlowly(); 

void motorBackward(int distance); 

void curveRight90(int distance); 

void curveLeft90(int distance);  

float ultraSonicSensor(); 

void obstacleRoutine(); 

void motorCountleft(); 

void motorCountRight(); 

void CountReset(); 

void lineFollowing(); 

bool sensorState(int i); 

int computeLineError(); 

void gripper(int pulse); 

void runMission(); 

void curveLeftAroundItSelf(int distance); 

void sendTelemetry(String botName, int leftSpeed, int rightSpeed, BotState state); 

void blinkStartingLights(); 

void movingForwardLights(); 

void movingLeftLights(); 

void movingRightLights(); 

void movingBackwordLights(); 

 

void setup() { 

 

  Serial.begin(9600); 

  // set motor pins as OUTPUT 

  pinMode(MOTORA1, OUTPUT); 

  pinMode(MOTORA2, OUTPUT); 

  pinMode(MOTORB1, OUTPUT); 

  pinMode(MOTORB2, OUTPUT); 

 

  pinMode(GRIPPER, OUTPUT); 

   

  pinMode(TRIG, OUTPUT); 

  pinMode(ECHO, INPUT); 

  pinMode(BUTTON1, INPUT); 

 

  pinMode(MOTORLEFTSENSOR, INPUT_PULLUP); 

  pinMode(MOTORRIGHTSENSOR, INPUT_PULLUP); 

 

  attachInterrupt(digitalPinToInterrupt(MOTORLEFTSENSOR),  motorCountleft, RISING); 

  attachInterrupt(digitalPinToInterrupt(MOTORRIGHTSENSOR), motorCountRight, RISING); 

 

  // set all the sensor pins as INPUT 

  for (int i = 0; i < 8; i++) { 

    pinMode(lineSensorPins[i], INPUT); 

    thresholds[i] = 515; // fixed number for black detection 

  } 

 

  //------------NeoPixel------------------ 

  pixels.begin(); 

  pixels.show(); // Initialize all pixels to off 

 

  // initial state 

  motorStop(); 

  gripper(GRIPPEROPEN); // open the gripper at the start 

 

} 

 

void loop() { 

  //read the state of the button 

  buttonState = digitalRead(BUTTON1); 

  // check if the button is pressed 

  if (buttonState == LOW) { 

    motorStop(); // stop the robot 

  } else { 

    runMission(); 

  } 

   

} 

//------------ Motor Control ----------------- 

void motorStrongStop(){ 

  const int stopDuration = 1000; 

  unsigned long startTime = millis(); 

  while (millis() - startTime < stopDuration){ 

    digitalWrite(MOTORA1, HIGH); 

    digitalWrite(MOTORA2, HIGH); 

    digitalWrite(MOTORB1, HIGH); 

    digitalWrite(MOTORB2, HIGH); 

    leftSpeed = 0; 

    rightSpeed = 0; 

 

  } 

} 

void motorStop() { 

  const int stopDuration = 50; 

  unsigned long startTime = millis(); 

  while (millis() - startTime < stopDuration){ 

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, 0); 

    analogWrite(MOTORB1, 0); 

    analogWrite(MOTORB2, 0); 

    leftSpeed = 0; 

    rightSpeed = 0; 

  } 

} 

void motorForwardSlowly(){ 

   

  const int SPEEDMOTORA = 200; 

  const int SPEEDMOTORB = 175; 

 

  // left wheel 

  analogWrite(MOTORA1, 0); 

  analogWrite(MOTORA2, SPEEDMOTORA); 

 

  // Right wheel 

  analogWrite(MOTORB1, SPEEDMOTORB); 

  analogWrite(MOTORB2, 0); 

 

  // update the global speed 

  leftSpeed = SPEEDMOTORA; 

  rightSpeed = SPEEDMOTORB; 

} 

void motorForward(int distance) { 

  CountReset();  // reset encoders 

 

  const int BASE_LEFT  = 225;   // base PWM for left motor 

  const int BASE_RIGHT = 195;   // base PWM for right motor 

  const float K = 0.3;          // proportional gain (tuneable) 

 

  while (motorLeft_count <= distance || motorRight_count <= distance) { 

    // Difference between wheels 

    long diff = motorLeft_count - motorRight_count; 

 

    // Apply proportional correction 

    int left  = BASE_LEFT  - (diff * K); 

    int right = BASE_RIGHT + (diff * K); 

 

    left = constrain(left, 0, 225); 

    right = constrain(right, 0, 195); 

 

    // Drive motors forward 

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, left); 

    analogWrite(MOTORB1, right); 

    analogWrite(MOTORB2, 0); 

 

    // update the global speed 

    leftSpeed = left; 

    rightSpeed = right; 

  } 

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

 

    // update the global speed 

    leftSpeed = SPEEDMOTORA; 

    rightSpeed = SPEEDMOTORB; 

 

    //LED 

    movingBackwordLights(); 

 

  } 

}  

 

void curveRight90(int distance) { 

  CountReset(); 

  const int SPEEDMOTORA = 255; 

  const int SPEEDMOTORB = 100; 

  while (motorRight_count <= distance && motorLeft_count <= distance){ 

      

    // Left wheel  

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, SPEEDMOTORA); 

    // Right wheel  

    analogWrite(MOTORB1, SPEEDMOTORB); 

    analogWrite(MOTORB2, 0); 

 

    // update the global speed 

    leftSpeed = SPEEDMOTORA; 

    rightSpeed = SPEEDMOTORB; 

 

    //LED 

    movingRightLights(); 

 

  } 

} 

void curveLeftAroundItSelf(int distance) { 

  CountReset(); 

  const int SPEEDMOTORA = 0; 

  const int SPEEDMOTORB = 255; 

  while (motorRight_count <= distance && motorLeft_count <= distance){ 

     

    // Left wheel  

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, SPEEDMOTORA); 

 

    // Right wheel 

    analogWrite(MOTORB1, SPEEDMOTORB); 

    analogWrite(MOTORB2, 0); 

 

    // update the global speed 

    leftSpeed = SPEEDMOTORA; 

    rightSpeed = SPEEDMOTORB; 

 

    // LED 

    movingLeftLights(); 

  } 

} 

 

void curveLeft90(int distance) { 

  CountReset(); 

  const int SPEEDMOTORA = 100; 

  const int SPEEDMOTORB = 255; 

  while (motorRight_count <= distance && motorLeft_count <= distance){ 

     

    // Left wheel  

    analogWrite(MOTORA1, 0); 

    analogWrite(MOTORA2, SPEEDMOTORA); 

 

    // Right wheel 

    analogWrite(MOTORB1, SPEEDMOTORB); 

    analogWrite(MOTORB2, 0); 

 

    // update the global speed 

    leftSpeed = SPEEDMOTORA; 

    rightSpeed = SPEEDMOTORB; 

 

    //LED 

    movingLeftLights(); 

  } 

} 

 

float ultraSonicSensor(){    // this will measure the distance to an object 

 

  static unsigned long lastPingTime = 0; 

  static float lastValidDistance = -1; 

  unsigned long currentTime = millis(); 

  // only send new ping if the enough time has passed 

  if (currentTime - lastPingTime >= 150) { // 150 ms between real pings 

 

    float duration, distance; 

 

    digitalWrite(TRIG, LOW); 

    delayMicroseconds(2); 

    digitalWrite(TRIG, HIGH); 

    delayMicroseconds(10); 

    digitalWrite(TRIG, LOW); 

 

    duration = pulseIn(ECHO, HIGH, 30000); // 30ms timeout after that it retrun 0 

 

    if(duration == 0){ 

      distance = -1; // no echo 

    } else { 

      distance = (duration*.0343) / 2; // in cm 

    } 

 

    // update the last valid reading and timer is we get a valid value 

    if (distance != -1){ 

      lastValidDistance = distance; 

    } 

    lastPingTime = currentTime; // update the timer  

  } 

  return lastValidDistance;  

} 

 

void obstacleRoutine() { 

 

  // Step 1: back up slightly 

  motorBackward(10); 

 

  // Step 2: initial left turn 

  curveLeft90(35); 

 

  // step 3 : turn right 

  curveRight90(60); 

  currentState = FOLLOW_LINE; 

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

 

if (totalWeight == 0) { 

    if (lastError > 0) { 

        return -10;   // turn left 

    } else if (lastError < 0) { 

        return 10;    // turn right 

    } else { 

        return 0; 

    } 

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

 

  const int BASE_SPEED = 220;   // base motor speed it was 150 

  const int CORRECTION = 295;    // correction factor *values can be tuned*  

 

  // Adjust speeds based on error 

  leftSpeed  = BASE_SPEED - (error * CORRECTION);  

  rightSpeed = BASE_SPEED + (error * CORRECTION);  

 

  // Clamp speeds to valid range 

  leftSpeed  = constrain(leftSpeed, 0, 220); 

  rightSpeed = constrain(rightSpeed, 0, 220); 

 

  // Drive motors with adjusted speeds 

  analogWrite(MOTORA1, 0); 

  analogWrite(MOTORA2, leftSpeed); 

  analogWrite(MOTORB1, rightSpeed); 

  analogWrite(MOTORB2, 0); 

 

  // LED 

  movingForwardLights(); 

} 

 

void gripper(int pulse){ 

  static unsigned long timer; // using static let you initialize the timer to zero 

  static long lastPulse; 

  if (pulse > 0) lastPulse = pulse; 

  if (millis() > timer) { 

    timer = millis() + 20;   // Sets the next time when a pulse should be sent. 

    digitalWrite(GRIPPER, 1); 

    delayMicroseconds(lastPulse); 

    digitalWrite(GRIPPER, 0); 

  } 

} 

void runMission(){ 

  switch (currentState) { 

    case STARTING_POINT: { 

      motorStop(); 

      blinkStartingLights(); 

 

      float distance = ultraSonicSensor(); 

 

      if (distance > STARTING_FLAG ) { 

      // when there is not obstacle 

      currentState = SEARCH_OBJECT; 

      obstacleCounter = 0; 

      } else { 

        // Obstacle detected → increment counter 

        obstacleCounter++; 

        if (obstacleCounter >= OBSTACLE_CONFIRM) { 

         motorStop();    

         obstacleCounter = 0;  

        } 

      } 

 

      sendTelemetry(botName, leftSpeed, rightSpeed, currentState); 

      break; 

    } 

 

    case SEARCH_OBJECT: { 

      movingForwardLights(); 

      gripper(GRIPPEROPEN); 

      motorForward(startingDistance); 

      static int blackCounter = 0;  

      // check if all sensors see black 

      bool allBlack = true; 

      for (int i = 0; i < 8; i++) { 

        if (!sensorState(i)){ 

          allBlack = false; 

          break; 

        } 

      } 

      if (allBlack){ 

        blackCounter++; 

      } else { 

        blackCounter = 0; 

      } 

      // check if we see black for x times or more 

      if (blackCounter >= 4){ 

        motorStop(); 

        gripper(GRIPPERCLOSE); 

        currentState = TURN_LEFT; 

        blackCounter = 0; // reset the counter for future  use 

      } 

      sendTelemetry(botName, leftSpeed, rightSpeed, currentState); 

      break; 

    } 

 

    case TURN_LEFT: { 

      motorStrongStop(); 

      curveLeftAroundItSelf(curveLeftAroundItSelfDistance); 

      currentState = FOLLOW_LINE;  

      sendTelemetry(botName, leftSpeed, rightSpeed, currentState); 

      break; 

    }  

 

    case FOLLOW_LINE: { 

      float distance = ultraSonicSensor(); 

 

      if (distance > OBSTACLE_THRESHOLD || distance == -1) { 

        // No obstacle → reset counter and follow line 

        obstacleCounter = 0; 

        lineFollowing(); 

 

        // --- Black line detection for drop zone --- 

        static int blackCounter = 0;  

        bool allBlack = true; 

        for (int i = 0; i < 8; i++) { 

          if (!sensorState(i)){ 

            allBlack = false; 

            break; 

          } 

        } 

        if (allBlack) { 

          blackCounter++; 

        } else { 

          blackCounter = 0; 

        } 

        if (blackCounter >= 10) { 

          currentState = DROP_OBJECT; 

          blackCounter = 0; 

        } 

 

      } else { 

        // Obstacle detected → increment counter 

        obstacleCounter++; 

        if (obstacleCounter >= OBSTACLE_CONFIRM) { 

          motorStrongStop(); 

          obstacleRoutine();    

          obstacleCounter = 0; // reset after avoidance 

        } 

      } 

      sendTelemetry(botName, leftSpeed, rightSpeed, currentState); 

      break; 

    } 

 

    case DROP_OBJECT: { 

      motorStrongStop(); // for 1 sec stop 

      motorBackward(8); 

      motorStrongStop(); 

      gripper(GRIPPEROPEN); 

      motorBackward(backwardDistance); 

      currentState = DONE; 

      sendTelemetry(botName, leftSpeed, rightSpeed, currentState); 

      break; 

    } 

 

    case DONE: { 

      motorStop(); 

 

      // Notify master that mission is finished 

      Serial.print(botName);  

      Serial.println(",DONE"); 

      sendTelemetry(botName, leftSpeed, rightSpeed, currentState); 

      break; 

    } 

  } 

} 

void sendTelemetry(String botName, int leftSpeed, int rightSpeed, BotState state) { 

  Serial.print(botName); 

  Serial.print(","); 

  Serial.print(leftSpeed); 

  Serial.print(","); 

  Serial.print(rightSpeed); 

  Serial.print(","); 

  Serial.println(stateToText(state)); 

} 

 

void blinkStartingLights() { 

  unsigned long now = millis(); 

 

  // Blink every 300 ms 

  if (now - lastBlinkTime >= 300) { 

    lastBlinkTime = now; 

    blinkState = !blinkState;  // toggle ON/OFF 

  } 

 

  if (blinkState) { 

    // ON state 

 

    // BACK LEDs (0,1) = red 

    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 

    pixels.setPixelColor(1, pixels.Color(0, 255, 0)); 

 

    // FRONT LEDs (2,3) = white (GRB format!) 

    pixels.setPixelColor(2, pixels.Color(255, 255, 255)); 

    pixels.setPixelColor(3, pixels.Color(255, 255, 255)); 

 

  } else { 

    // OFF state 

    pixels.clear(); 

  } 

  pixels.show(); 

} 

 

void movingForwardLights() { 

  pixels.clear(); 

  // Set back LEDs to yellow 

  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); 

  pixels.setPixelColor(1, pixels.Color(255, 255, 0));  

 

  // Set front LEDs to white 

  pixels.setPixelColor(2, pixels.Color(255, 255, 255));  

  pixels.setPixelColor(3, pixels.Color(255, 255, 255)); 

 

  pixels.show(); // Apply the colors 

} 

 

void movingLeftLights() { 

  unsigned long now = millis(); 

 

  // Blink every 300 ms 

  if (now - lastBlinkTime >= 300) { 

    lastBlinkTime = now; 

    blinkState = !blinkState;  // toggle ON/OFF 

  } 

 

  if (blinkState) { 

    // ON state 

 

    // BACK LEDs (0,1) = bluesh 

    pixels.setPixelColor(0, pixels.Color(0, 255, 255)); // left 

 

    // FRONT LEDs (2,3) = blush (GRB format!) 

    pixels.setPixelColor(3, pixels.Color(255, 255, 255)); // left  

 

  } else { 

    // OFF state 

    pixels.clear(); 

  } 

  pixels.show(); 

} 

 

void movingRightLights() { 

  unsigned long now = millis(); 

 

  // Blink every 300 ms 

  if (now - lastBlinkTime >= 300) { 

    lastBlinkTime = now; 

    blinkState = !blinkState;  // toggle ON/OFF 

  } 

 

  if (blinkState) { 

    // ON state 

 

    // BACK LEDs (0,1) = bluesh 

    //pixels.setPixelColor(0, pixels.Color(0, 255, 255)); // left 

    pixels.setPixelColor(1, pixels.Color(0, 255, 255)); // right 

 

    // FRONT LEDs (2,3) = blush (GRB format!) 

    pixels.setPixelColor(2, pixels.Color(0, 255, 255)); // right 

    //pixels.setPixelColor(3, pixels.Color(255, 255, 255)); // left  

 

  } else { 

    // OFF state 

    pixels.clear(); 

  } 

  pixels.show(); 

} 

 

void movingBackwordLights() { 

  unsigned long now = millis(); 

 

  // Blink every 300 ms 

  if (now - lastBlinkTime >= 300) { 

    lastBlinkTime = now; 

    blinkState = !blinkState;  // toggle ON/OFF 

  } 

 

  if (blinkState) { 

    // ON state 

 

    // BACK LEDs (0,1) = red 

    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // left 

    pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // right 

 

  } else { 

    // OFF state 

    pixels.clear(); 

  } 

  pixels.show(); 

}