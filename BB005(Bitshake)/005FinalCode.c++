#include <Adafruit_NeoPixel.h>

// neopixel pins
#define NEOPIXEL_PIN 4
#define NEOPIXEL_NUM 4

// motor corrections for startup sequence
#define L_CORRECTION 20
#define R_CORRECTION 0

// motor pins
#define MOTOR_A1 9
#define MOTOR_A2 10
#define MOTOR_B1 6
#define MOTOR_B2 5

// sensor pins
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6
#define S8 A7

// line sensing values
#define BLACK_TO_WHITE 700 // below 700 -> switch to white
#define WHITE_TO_BLACK 800 // above 800 -> switch to black

// motor speeds and how strong the opposing wheels slow when turning
#define MAX_SPEED 255 // 255
#define TURN_STR_1 60 // 50
#define TURN_STR_2 120  // 120
#define TURN_STR_3 160  // 180

// values affecting the turning logic
#define DEF_TURN_START 200
#define TURN_REST 800
#define EMPTY_TIME 250

// gripper valuesbotName
#define GRIPPER 12
#define GRIPPER_OPEN 1800
#define GRIPPER_CLOSE 1050

// sound sensor pins
#define TRIG_PIN 11
#define ECHO_PIN 13

// wheel interrupt pins
#define IRT_1 2
#define IRT_2 3

// distance to object to turn around
#define DEF_OBJECT_DISTANCE 140

// botname
String botName = "BitShake";




// variables

int sensors[] = {S1, S2, S3, S4, S5, S6, S7, S8};
bool sensorActive[] = {false, false, false, false, false, false, false, false};
uint32_t sensorTimer;

uint32_t turnStart;
uint32_t turnRestTimer;
uint32_t emptyTimer;

uint32_t leftSpeed;
uint32_t rightSpeed;

bool leftPath;
bool rightPath;
bool empty;
bool leftPathPast;
bool rightPathPast;
bool emptyPast;
bool center;

bool turn_left;
bool turning;

bool line;
bool line_past;
uint32_t lineTimer;

int blockNum;
bool blockLatch;

bool running;

uint32_t echo_timer;
uint16_t distance;

uint32_t commsTimer;

uint32_t stuckTimer;

bool neoBlink;
uint32_t neoTimer;
uint32_t neoColors[4];
Adafruit_NeoPixel neoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);



// function headers

void go();
void straight();
void turn();
void paths();
void blocks();

void move(int left, int right);
void motors();
void gripper(int pulse);
void neoPixels();

void readSensors();
void echoSensors();
void checkStuck();

void leftIRT();
void rightIRT();

void checkComms();
void comms(String botstatus);
void sendTelemetry(String botname, int leftSpeed, int rightSpeed, String state);


// functions

void setup() {
  //motor pins setup
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  // motor default status
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);

  // sound sensor pins setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // motor interrupt pins setup
  pinMode(IRT_1, INPUT);
  pinMode(IRT_2, INPUT);

  // // motor interrupts setup
  attachInterrupt(digitalPinToInterrupt(IRT_1), leftIRT, FALLING ); // IRQ
  attachInterrupt(digitalPinToInterrupt(IRT_2), rightIRT, FALLING ); // maybe try changing to CHANGE

  // default values

  running = false; // when checking for signals just set this to false by default, then comms turn the bot on

  sensorTimer = millis();
  turnStart = millis();
  turn_left = true;
  turning = false;
  turnRestTimer = millis();

  leftSpeed = 0;
  rightSpeed = 0;

  leftPath = false;
  rightPath = false;
  empty = false;
  center = false;

  rightPathPast = false;
  leftPathPast = false;
  emptyPast = false;

  lineTimer = millis();
  emptyTimer = millis();
  line = false;
  line_past = false;

  blockNum = 0;
  blockLatch = false;

  commsTimer = millis();
  stuckTimer = millis();

  neoTimer = millis();
  neoBlink = true;
  for(uint8_t pixel = 0; pixel < neoPixel.numPixels(); pixel++)
    neoPixel.setPixelColor(pixel, 0, 0, 255);

  readSensors(); // initial sensor reading

  Serial.begin(9600); // serial innit

  neoPixel.begin(); // neopixel innit
  neoPixel.show();
}

// main loop
void loop() {
  readSensors();

  if(running) // after callout
  {
    blocks();
    if(blockNum == 0) // before first block
    {
      comms("starting");

      gripper(GRIPPER_OPEN);

      readSensors();

      bool left_ss = (sensorActive[0] || sensorActive[1] || sensorActive[2]);
      bool right_ss = (sensorActive[5] || sensorActive[6] || sensorActive[7]);

      if(left_ss && !right_ss)
        move(MAX_SPEED, MAX_SPEED-80);
      else if(right_ss && !left_ss)
        move(MAX_SPEED-80, MAX_SPEED);
      else
        move(MAX_SPEED - L_CORRECTION, MAX_SPEED - R_CORRECTION);
    }
    else if(blockNum == 1) // after first block
    {
      gripper(GRIPPER_CLOSE);
      go(); // main function
    }
  }
  else // waiting for callout
  {
    move(0, 0);
    checkComms();
  }

  neoPixels();
}

// interrupts to check if the bot is stuck
void leftIRT() {
  stuckTimer = millis()+200;
}

void rightIRT() {
  stuckTimer = millis()+200;
}

// main function
void go() {
  readSensors();
  echoSensors();

  if(turning) { // before end of turning + start
    comms("turning");
    turnRestTimer = millis()+TURN_REST;

    if(millis() > turnStart)
      turn();
    else {
      leftSpeed = MAX_SPEED;
      rightSpeed = MAX_SPEED;
    }
  }
  else { // following line
    comms("running");

    checkStuck();
    
    straight();
    paths();

    neoBlink = false;
    for(uint8_t pixel = 0; pixel < neoPixel.numPixels(); pixel++)
      neoPixel.setPixelColor(pixel, 0, 255, 0);
  }

  motors(); // set motor speed and such
}

// backs up if the bot wasnt moving for long enough
void checkStuck() {
  while(millis() > stuckTimer) { // didnt get IRT for 50+ ms
    move(-MAX_SPEED, -MAX_SPEED);
  }
}

// looks for black squares (blocks) and acts accordingly
void blocks() {
  line_past = line;

  if(sensorActive[0] && sensorActive[1] && sensorActive[2] && sensorActive[3] && sensorActive[4] && sensorActive[5] && sensorActive[6] && sensorActive[7])
    line = true;
  else
    line = false;

  if(line && running) { // on line
    leftSpeed = MAX_SPEED;
    rightSpeed = MAX_SPEED;
  }

  if(line && !line_past) // enters line
  {
    lineTimer = millis();
    blockLatch = false;
  }

  if(line && !blockLatch) { // on line with latch
    if(millis() - lineTimer > 200) // was on line for at least 50ms
    {
      blockLatch = true;
      blockNum++;
      if(blockNum == 1) { // get off first block
        neoBlink = true;
        neoPixel.setPixelColor(0, 255, 255, 0); // Left Rear
        neoPixel.setPixelColor(1, 0, 0, 0); // Right Rear
        neoPixel.setPixelColor(2, 0, 0, 0); // Right Front
        neoPixel.setPixelColor(3, 255, 255, 0);   // Left Front

        uint32_t gripper_timer = millis()+1000;
        while(millis() < gripper_timer) {
          gripper(GRIPPER_CLOSE);
          move(0, MAX_SPEED);

          neoPixels();
        }
      }
      else if(blockNum == 2) { // parking
        neoBlink = true;
        neoPixel.setPixelColor(0, 255, 0, 0); // Left Rear
        neoPixel.setPixelColor(1, 255, 0, 0); // Right Rear
        neoPixel.setPixelColor(2, 255, 0, 0); // Right Front
        neoPixel.setPixelColor(3, 255, 0, 0); // Left Front

        uint32_t gripper_timer = millis()+250;
        while(millis() < gripper_timer) {
          comms("parking");

          move(-MAX_SPEED, -MAX_SPEED);

          neoPixels();
        }

        gripper_timer = millis()+200;
        while(millis() < gripper_timer) {
          comms("parking");

          move(0, 0);
          gripper(GRIPPER_OPEN);

          neoPixels();
        }

        neoBlink = true;
        neoPixel.setPixelColor(0, 0, 0, 255); // Left Rear
        neoPixel.setPixelColor(1, 0, 0, 255); // Right Rear
        neoPixel.setPixelColor(2, 0, 0, 255); // Right Front
        neoPixel.setPixelColor(3, 0, 0, 255); // Left Front

        gripper_timer = millis()+1500;
        while(millis() < gripper_timer) {
          comms("parking");

          move(-MAX_SPEED, -MAX_SPEED);
          gripper(GRIPPER_OPEN);

          neoPixels();
        }
          
        running = false;

        neoBlink = false;
        neoPixel.setPixelColor(0, 0, 0, 0); // Left Rear
        neoPixel.setPixelColor(1, 0, 0, 0); // Right Rear
        neoPixel.setPixelColor(2, 0, 0, 0); // Right Front
        neoPixel.setPixelColor(3, 0, 0, 0); // Left Front

        for(int i = 0; i < 3; i++) {
          sendTelemetry(botName, leftSpeed, rightSpeed, "DONE");
          delay(20);
        }
      }
    }
  }
}

// checks for available paths at crossroads and turns accordingly
void paths() {
  leftPathPast = leftPath;
  rightPathPast = rightPath;
  emptyPast = empty;

  leftPath = false;
  rightPath = false;
  empty = (!sensorActive[0] && !sensorActive[1] && !sensorActive[2] && !sensorActive[3] && !sensorActive[4] && !sensorActive[5] && !sensorActive[6] && !sensorActive[7]);
  center = (sensorActive[2] || sensorActive[3] || sensorActive[4] || sensorActive[5]);

  if(sensorActive[6] && sensorActive[7])
    leftPath = true;
  if(sensorActive[0] && sensorActive[1])
    rightPath = true;

  if(!empty)
    emptyTimer = millis();

  if(millis() > turnRestTimer) {
    if(!rightPath && rightPathPast) { // priority 1: right turns
      turnStart = millis()+DEF_TURN_START;
      turning = true;
      turn_left = false;
    }
    else if(!leftPath && leftPathPast) {// priority 2: left turns
      if(!center) { // if cannot go straight
        turnStart = millis()+DEF_TURN_START;
        turning = true;
        turn_left = true;
      }
    }
    else if(empty) {// priority 3: i havent got a clue
      if(!emptyPast)
        emptyTimer = millis()+EMPTY_TIME;

      if(millis() > turnRestTimer) {
        turnStart = millis()+DEF_TURN_START;
        turning = true;
      }
    }
  }
}

// operates the gripper
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

// set of instructions for continuous turning
void turn() {
  if(turn_left) { // left
    if(sensorActive[6])
      turning = false;

    leftSpeed = -MAX_SPEED;
    rightSpeed = MAX_SPEED;

    neoBlink = true;
    neoPixel.setPixelColor(0, 255, 255, 0); // Left Rear
    neoPixel.setPixelColor(1, 0, 0, 0);     // Right Rear
    neoPixel.setPixelColor(2, 0, 0, 0);     // Right Front
    neoPixel.setPixelColor(3, 255, 255, 0); // Left Front
  }
  else { // right
    if(sensorActive[1])
      turning = false;

    leftSpeed = MAX_SPEED;
    rightSpeed = -MAX_SPEED;

    neoBlink = true;
    neoPixel.setPixelColor(0, 0, 0, 0);     // Left Rear
    neoPixel.setPixelColor(1, 255, 255, 0); // Right Rear
    neoPixel.setPixelColor(2, 255, 255, 0); // Right Front
    neoPixel.setPixelColor(3, 0, 0, 0);     // Left Front
  }
}

// follows the black line (straight)
void straight() {
  if(sensorActive[3] || sensorActive[4]) { // middle
    leftSpeed = MAX_SPEED;
    rightSpeed = MAX_SPEED;
  }
  else if(sensorActive[0]) { // -3
    leftSpeed = MAX_SPEED;
    rightSpeed = MAX_SPEED - TURN_STR_3;
  }
  else if(sensorActive[7]) { // +3
    leftSpeed = MAX_SPEED - TURN_STR_3;
    rightSpeed = MAX_SPEED;
  }
  else if(sensorActive[1]) { // -2
    leftSpeed = MAX_SPEED;
    rightSpeed = MAX_SPEED - TURN_STR_2;
  }
  else if(sensorActive[6]) { // +2
    leftSpeed = MAX_SPEED - TURN_STR_2;
    rightSpeed = MAX_SPEED;
  }
  else if(sensorActive[2]) { // -1
    leftSpeed = MAX_SPEED;
    rightSpeed = MAX_SPEED - TURN_STR_1;
  }
  else if(sensorActive[5]) { // +1
    leftSpeed = MAX_SPEED - TURN_STR_1;
    rightSpeed = MAX_SPEED;
  }
}

// operates the motors
void motors() {
  move(leftSpeed, rightSpeed); // there used to be a system that gradually increased/decreased the speed to these values but had to be removed
}

// reads the optical sensors
void readSensors() {
  if(millis() - sensorTimer >= 50) { // every 50ms
    sensorTimer = millis();

    for(int i = 0; i <= 7; i++) { // 8 sensors
      int sensor_value = analogRead(sensors[i]); // read current sensor

      if(sensorActive[i]) { // was black
        if(sensor_value <= BLACK_TO_WHITE) // switch
          sensorActive[i] = false;
      }
      else { // was white
        if(sensor_value >= WHITE_TO_BLACK) // switch
          sensorActive[i] = true;
      }
    }
  }
}

// translates the movement into pin controll (and adds limits)
void move(int left, int right) {
  // limits
  if(left > MAX_SPEED)
    left = MAX_SPEED;
  if(right > MAX_SPEED)
    right = MAX_SPEED;

  if(left < -MAX_SPEED)
    left = -MAX_SPEED;
  if(right < -MAX_SPEED)
    right = -MAX_SPEED;

  // left motor
  if(left >= 0) { // forward
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, left);
  }
  else { // backward
    analogWrite(MOTOR_A1, -left);
    analogWrite(MOTOR_A2, 0);
  }

  // right motor
  if(right >= 0) { // forward
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, right);
  }
  else { // backward
    analogWrite(MOTOR_B1, -right);
    analogWrite(MOTOR_B2, 0);
  }
}

// reads the sound sensors and translates into distance
void echoSensors() {
  if(millis() >= echo_timer) { // every 200ms
    echo_timer = millis()+200;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long int duration = pulseIn(ECHO_PIN, HIGH, 5000); // , 5000
    distance = (343*(duration/2))/1000;

    if((distance != 0) && (distance <= DEF_OBJECT_DISTANCE)) {
      turnStart = millis();
      turning = true;
    }

    Serial.println(distance);
  }
}

// controlls the neopixels
void neoPixels() {
  if(millis() >= neoTimer) { // every 200ms update
    neoTimer = millis()+200;

    if(neoBlink && (millis()%400 >= 200)) { // if blinking: every 400ms for 200ms
      for(uint8_t pixel = 0; pixel < neoPixel.numPixels(); pixel++)
        neoColors[pixel] = neoPixel.getPixelColor(pixel);

      neoPixel.clear();
      neoPixel.show();

      for(uint8_t pixel = 0; pixel < neoPixel.numPixels(); pixel++)
        neoPixel.setPixelColor(pixel, neoColors[pixel]);

    }
    else
      neoPixel.show();
  }
}

// sending data to master
void comms(String botstatus) {
  if(millis() > commsTimer) {
    sendTelemetry(botName, leftSpeed, rightSpeed, botstatus);
    commsTimer = millis()+250;
  }
}

// structuring data for sending
void sendTelemetry(String botname, int leftSpeed, int rightSpeed, String state) {
  Serial.print(botname);
  Serial.print(",");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.print(rightSpeed);
  Serial.print(",");
  Serial.println(state);
}

// listening for callout
void checkComms() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');  
    msg.trim();

    if (msg == botName + ",START"){
      running = true; // allow the mission to begin
      Serial.println(botName + String(",RECEIVED")); // confirmation
    }
  }
}