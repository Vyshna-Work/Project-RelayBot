// Sensor pins
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Motor pins
const int MOTORA1 = 6;  // Left Backwards
const int MOTORA2 = 5;  // Left Forwards
const int MOTORB1 = 10; // Right Forwards
const int MOTORB2 = 9;  // Right Backwards

const int threshold = 700;

const int Speed1 = 255;
int Speed2 = 150;
const int NOSPEED = 0;

bool left_mem;
bool right_mem;

void setup() {
  Serial.begin(9600);

  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORB2, OUTPUT);
}

void loop() {
  int sensorValues[8];

  // Read all sensors
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // value printing for debugging
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }

  Serial.print(" | B/W: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i] > threshold ? "B " : "W ");
  }
  Serial.println();

  // Determine line position
  bool Center = (sensorValues[3] > threshold) || (sensorValues[4] > threshold);
  bool Left   = (sensorValues[5] > threshold) || (sensorValues[6] > threshold) || (sensorValues[7] > threshold);
  bool Right  = (sensorValues[0] > threshold) || (sensorValues[1] > threshold) || (sensorValues[2] > threshold);

  //determineturning ratio
  if((sensorValues[5] > threshold) || (sensorValues[2] > threshold))
    Speed2 = 200;
  if((sensorValues[6] > threshold) || (sensorValues[1] > threshold))
    Speed2 = 150;
  if((sensorValues[7] > threshold) || (sensorValues[0] > threshold))
    Speed2 = 50;

  // Movement logic
  if (Center) {
    motorForward();
  } 
  else if (Left) {
    leftTurn();
    left_mem = true;
    right_mem = false;
  } 
  else if (Right) {
    rightTurn();
    right_mem = true;
    left_mem = false;
  } 
  else {
    // motorStop();
    if(left_mem)
    {
      leftTurn();
    }
    else if (right_mem)
    {
      rightTurn();
    }
  }

  delay(50);
}

// Motor functions
void motorStop() {
  digitalWrite(MOTORA1, LOW);
  digitalWrite(MOTORA2, LOW);
  digitalWrite(MOTORB1, LOW);
  digitalWrite(MOTORB2, LOW);
}

void motorForward() {
  analogWrite(MOTORA2, Speed1);
  analogWrite(MOTORB1, Speed1);
  analogWrite(MOTORA1, NOSPEED);
  analogWrite(MOTORB2, NOSPEED);
}

void leftTurn() {
  analogWrite(MOTORA2, Speed2);
  analogWrite(MOTORA1, NOSPEED);
  analogWrite(MOTORB1, Speed1);
  analogWrite(MOTORB2, NOSPEED);
}

void rightTurn() {
  analogWrite(MOTORA2, Speed1);
  analogWrite(MOTORA1, NOSPEED);
  analogWrite(MOTORB1, Speed2);
  analogWrite(MOTORB2, NOSPEED);
}
