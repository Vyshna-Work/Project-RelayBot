#include <time.h>
#include <Arduino.h>

#define A1 9
#define A2 10
#define B1 6
#define B2 5

#define L_speed_max 200
#define R_speed_max 200
#define meter_time 5000

#define BTN_1 11
#define BTN_2 12
#define BTN_3 4

#define IRT_1 2
#define IRT_2 3

#define TRIG_PIN 8
#define ECHO_PIN 13

uint32_t timer;
uint32_t main_timer;
uint32_t action_timer;
uint32_t display_timer;
uint32_t sense_timer;
uint32_t avoidance_timer;
uint32_t back_timer;
uint32_t L_timer;
uint32_t R_timer;

uint16_t Distance;
uint16_t Distance_old;

uint32_t L_count;
uint32_t R_count;

int32_t L_speed;
int32_t R_speed;

uint32_t L_aim;
uint32_t R_aim;

uint8_t mode;
bool avoidance;

void setup() {
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);

  pinMode(BTN_1, INPUT);
  pinMode(BTN_2, INPUT);
  pinMode(BTN_3, INPUT);

  pinMode(IRT_1, INPUT);
  pinMode(IRT_2, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IRT_1), Left_IRT, FALLING ); // IRQ
  attachInterrupt(digitalPinToInterrupt(IRT_2), Right_IRT, FALLING ); // maybe try changing to CHANGE

  L_count = 0;
  R_count = 0;

  L_timer = 0;
  R_timer = 0;

  L_speed = 0;
  R_speed = 0;

  Distance = 0;

  srand(time(NULL)); // random innit

  Serial.begin(9600);

  delay(500);
  
  mode = 0;
  avoidance = false;

  timer = millis();
  main_timer = millis();
  action_timer = millis();
  display_timer = millis();
  sense_timer = millis();
  avoidance_timer = millis();
  back_timer = millis();
}

void Left_IRT() {
  if(millis() - L_timer > 20) {
    L_timer = millis();
    L_count++;
  }
}

void Right_IRT() {
  if(millis() - R_timer > 20)
  {
    R_timer = millis();
    R_count++;
  }
}



void loop() {
  motors();
  display();
  sensors();

  run();
}



// void check_distance()
// {
//   if(Distance != 0) // sensors sense something
//   {
//     if((Distance < 50))
//     {
//       equal_back();
//     }
//     else if((Distance < 100))
//     {
//       stop();
//     }
//     else if ((Distance < 200))
//     {
//       if(Distance_old > Distance +10)
//       {
//         slow();
//       }
//     }    
//   }
//   else
//   {
//     equal_straight();  
//   }
// }

void run() {
  if(avoidance)
  {
    sensors();
    if((Distance_old > Distance+5) && (Distance < 200))
    {
      avoidance_timer = millis();
    }

    mode = 2;

    if(millis() - avoidance_timer < 250) // 500ms
    {
      L_aim = L_speed_max;
      R_aim = 0;
    }
    else if(millis() - avoidance_timer < 1500) // 2000ms
    {
      L_speed = L_speed_max;
      L_aim = L_speed_max;
      R_speed = R_speed_max;
      R_aim = R_speed_max;
    }
    else if(millis() - avoidance_timer < 3000 + (avoidance_timer-back_timer)) // 1000ms
    {
      R_aim = R_speed_max;
      L_aim = 0;
    }
    else if(millis() - avoidance_timer < 4500 + (avoidance_timer-back_timer))
    {
      L_speed = L_speed_max;
      L_aim = L_speed_max;
      R_speed = R_speed_max;
      R_aim = R_speed_max;
    }
    else if(millis() - avoidance_timer < 4800 + (avoidance_timer-back_timer))
    {
      L_aim = L_speed_max;
      R_aim = 0;
    }
    else
      avoidance = false;
  }
  else
  {
    if(Distance != 0) // sensors sense something
    {
      if((Distance < 50))
      {
        stop();
      }
      else if((Distance_old > Distance+5) && (Distance < 200))
      {
        avoidance = true;
        avoidance_timer = millis();
        back_timer = millis();
      }
    }
    else
    {
      equal_straight();  
    }
  }
}

void sensors()
{
  if(millis() - sense_timer > 50)
  {
    sense_timer = millis();

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long int duration = pulseIn(ECHO_PIN, HIGH, 5000);
    Distance_old = Distance;
    Distance = (343*(duration/2))/1000;
  }
}

void display()
{
  if(millis() - display_timer > 200)
  {
    display_timer = millis();

    Serial.println("============");

    Serial.print("Activity: ");
    switch (mode)
    {
      case 0:
        Serial.println("STOP");
        break;

      case 1:
        Serial.println("GO");
        break;

      case 2:
        Serial.println("AVOID");
        break;
      
      case 3:
        Serial.println("SLOW");
        break;
    }

    Serial.print("L_count: ");
    Serial.println(L_count);
    Serial.print("R_count: ");
    Serial.println(R_count);

    Serial.print("Echo: ");
    Serial.print(Distance);
    Serial.println(" mm");
  }
}

void motors()
{
  move(L_speed, R_speed);

  if(millis() - main_timer > 1)
  {
    main_timer = millis();

    if(L_speed < L_aim)
      L_speed++;
    else if(L_speed > L_aim)
      L_speed--;

    if(R_speed < R_aim)
      R_speed++;
    else if(R_speed > R_aim)
      R_speed--;
  }
}

void equal_straight()
{
  mode = 1;

  if(L_count > R_count)
  {
    L_aim = L_speed_max/1.3;
    R_aim = R_speed_max;
  }
  else if(R_count > L_count)
  {
    L_aim = L_speed_max;
    R_aim = R_speed_max/1.3;
  }
  else
  {
    L_aim = L_speed_max;
    R_aim = R_speed_max;
  }
}

// void equal_back()
// {
//   mode = 2;

//   if(L_count > R_count)
//   {
//     L_aim = -L_speed_max/1.3;
//     R_aim = -R_speed_max;
//   }
//   else if(R_count > L_count)
//   {
//     L_aim = -L_speed_max;
//     R_aim = -R_speed_max/1.3;
//   }
//   else
//   {
//     L_aim = -L_speed_max;
//     R_aim = -R_speed_max;
//   }
// }

void stop()
{
  mode = 0;

  L_aim = 0;
  R_aim = 0;

  L_count = 0;
  R_count = 0;
}

void slow()
{
  mode = 3;

  L_aim = L_speed_max/1.3;
  R_aim = R_speed_max/1.3;
}

void move(int left, int right)
{
  if(left >= 0)
  {
    analogWrite(A1, 0);
    analogWrite(A2, left);
  }
  else
  {
    analogWrite(A1, -left);
    analogWrite(A2, 0);
  }

  if(right >= 0)
  {
    analogWrite(B1, 0);
    analogWrite(B2, right);
  }
  else
  {
    analogWrite(B1, -right);
    analogWrite(B2, 0);
  }
}