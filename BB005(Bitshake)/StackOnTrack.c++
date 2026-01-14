and then i have this what i think is the  midterm race:
#include <time.h>

#define DEBUG

#define A1 9
#define A2 10
#define B1 6
#define B2 5

#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6
#define S8 A7

#define BLACK_TO_WHITE 700 // below 700 -> switch to white
#define WHITE_TO_BLACK 800 // above 800 -> switch to black

// speed = MAX_SPEED - TURN_STRENGTH * (i^TURN_POW)
#define MAX_SPEED 254 // max: 250
// #define TURN_STRENGTH 50 // default: 50 
// #define TURN_POW 1.2 // default: 1.2
#define TURN_STR_1 60 // 50
#define TURN_STR_2 120 // 120
#define TURN_STR_3 200 // 180

#define CORRECTION_TIME 1 // default: 1           how long to increment/decrement speed to required value
#define CORRECTION_VALUE 2 // default: 1          how much does the system increment/decrement

#define TRIG_PIN 12
#define ECHO_PIN 13

#define BTN_1 7

#define DEFAULT_LINES_CROSSED 1 // change to 1 if no black squares

int32_t L_speed;
int32_t R_speed;

uint32_t L_aim;
uint32_t R_aim;

uint32_t Limit_speed;

uint32_t display_timer;
uint32_t sensor_timer;
uint32_t echo_timer;
uint32_t main_timer;

uint16_t Distance;

uint32_t line_timer;
// uint32_t run_timer;

int Sensors[] = {S1, S2, S3, S4, S5, S6, S7, S8};

// bool Sensor_past[8];
bool Sensor_active[] = {false, false, false, false, false, false, false, false};

bool running;
bool line;
bool line_past;
int lines_crossed;
int block_num;
bool block_latch;
bool activated;

uint32_t track_timer;
uint32_t track_time;
uint32_t btn_timer;

// int Sensor_values[8];

// void activation();

void setup() {
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);

  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(B1, LOW);
  digitalWrite(B2, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);

  delay(500);

  display_timer = millis();
  sensor_timer = millis();
  echo_timer = millis();
  main_timer = millis();
  btn_timer = millis();

  line_timer = millis();
  block_num = 0;
  block_latch = false;
  // run_timer = 0;

  running = true;
  activated = false;
  line = false;
  line_past = false;
  lines_crossed = DEFAULT_LINES_CROSSED;

  track_timer = millis();
  track_time = 0;

  running = true;

  Limit_speed = MAX_SPEED;

  Distance = 0;

  pinMode(BTN_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_1), activation, FALLING); // IRQ
}


void loop() {
  read_sensors();
  // echo_sensors();
  display();

  if(activated)
  {
    go();
    blocks();
    motors();    
  }
  else
  {
    if(digitalRead(BTN_1) == LOW)
    {
      activated = true;
    }

    block_num = 0;
    block_latch = false;
    
    line = false;
    line_past = false;
    lines_crossed = DEFAULT_LINES_CROSSED; // CHANGE TO 1 IF THERE IS NO BLACK SQUARE!!!

    track_timer = millis();
    track_time = 0;

    running = true;
  }
}

void activation()
{
  activated = true;
  // if(millis() - btn_timer > 200)
  // {
  //   btn_timer = millis();
  //   activated = !activated;
  // }
}

/*

S1 - 3 = 200
S2 - 2 = 100
S3 - 1 = 50
S4 - 0 = 0

S5 - 0 = 0
S6 - 1 = 50
S7 - 2 = 100
S8 - 3 = 200

*/

void go()
{
  line_past = line;
  // line = true;

  // for(int i = 0; i <= 7; i++) // 8 sensors
  // {
  //   if(i <= 3) // left sensors
  //   {
  //     if(Sensor_active[3-i])
  //     {
  //       R_aim = Limit_speed - (TURN_STRENGTH*pow(i, TURN_POW)); // (50*pow(i, 1.2));
  //       L_aim = Limit_speed;     
  //     }
  //     else
  //       line = false;
  //   }
  //   else // right sensors
  //   {
  //     if(Sensor_active[i])
  //     {
  //       L_aim = Limit_speed - (TURN_STRENGTH*pow((i-4), TURN_POW));
  //       R_aim = Limit_speed;
  //     }
  //     else
  //       line = false;
  //   }
  // }

  if(Sensor_active[0] && Sensor_active[1] && Sensor_active[2] && Sensor_active[3] && Sensor_active[4] && Sensor_active[5] && Sensor_active[6] && Sensor_active[7])
    line = true;
  else
    line = false;

  if(Sensor_active[3] || Sensor_active[4]) // middle
  {
    L_aim = Limit_speed;
    R_aim = Limit_speed;

    // if(L_speed < L_aim)
    //   L_speed++;
    // if(L_speed < L_aim)
    //   L_speed++;
  }
  if(Sensor_active[0]) // -3
  {
    L_aim = Limit_speed;
    R_aim = Limit_speed - TURN_STR_3;
  }
  if(Sensor_active[1]) // -2
  {
    L_aim = Limit_speed;
    R_aim = Limit_speed - TURN_STR_2;
  }
  if(Sensor_active[2]) // -1
  {
    L_aim = Limit_speed;
    R_aim = Limit_speed - TURN_STR_1;
  }
  if(Sensor_active[5]) // +1
  {
    L_aim = Limit_speed - TURN_STR_1;
    R_aim = Limit_speed;
  }
  if(Sensor_active[6]) // +2
  {
    L_aim = Limit_speed - TURN_STR_2;
    R_aim = Limit_speed;
  }
  if(Sensor_active[7]) // +3
  {
    L_aim = Limit_speed - TURN_STR_3;
    R_aim = Limit_speed;
  }

  if(L_aim < 0)
    L_aim = 0;
  else if(L_aim > Limit_speed)
    L_aim = Limit_speed;
  
  if(R_aim < 0)
    R_aim = 0;
  else if(R_aim > Limit_speed)
    R_aim = Limit_speed;

  if(!running)
  {
    L_aim = 0;
    R_aim = 0;
  }
}



void blocks()
{
  if(line && running) // on line
  {
    L_aim = Limit_speed;
    R_aim = Limit_speed;
  }

  if(line && !line_past) // enters line
  {
    line_timer = millis();
    block_latch = false;
    if(running)
    {
      L_speed = (L_speed+R_speed)/2;
      R_speed = L_speed;
    }
  }

  // if(!line && line_past) // exits line
  // {
  //   if(block_num == 1 &&)
  //   {
  //     track_timer = millis();
  //   }
  // }

  if(line && !block_latch) // on line with latch
  {
    if(millis() - line_timer > 200) // was on line for at least 50ms
    {
      block_latch = true;
      block_num++;
      if(block_num == 1)
      {
        track_timer = millis();
      }
      else if(block_num == 2)
      {
        running = false;
        track_time = millis() - track_timer;
      }
    }
  }
}



void motors()
{
  move(L_speed, R_speed);

  if(millis() - main_timer > CORRECTION_TIME)
  {
    main_timer = millis();

    if(L_speed < L_aim)
      L_speed += CORRECTION_VALUE;
    else if(L_speed > L_aim)
      L_speed -= CORRECTION_VALUE;

    if(R_speed < R_aim)
      R_speed += CORRECTION_VALUE;
    else if(R_speed > R_aim)
      R_speed -= CORRECTION_VALUE;

    if(L_speed < 0)
      L_speed = 0;
    if(R_speed < 0)
      R_speed = 0;

    if(Sensor_active[3] || Sensor_active[4]) // middle
    {
      if(L_speed < L_aim)
        L_speed+=2;
      if(R_speed < R_aim)
        R_speed+=2;
    }
  }
}

void move(int left, int right)
{
  if(left > 255)
    left = 255;
  if(right > 255)
    right = 255;

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

// void display()
// {
//   if(millis() - display_timer >= 250) // every 250ms
//   {
//     display_timer = millis();

//     Serial.println("\n\n\n\n\n\n\n============");

//     if(block_num == 0)
//     {
//       Serial.println("Waiting for the starting line...");
//     }
//     else if(block_num == 1)
//     {
//       track_time = millis()-track_timer;

//       Serial.print("t: ");
//       Serial.print(track_time/1000);
//       Serial.print(".");
//       Serial.println((track_time%1000)/10);
//       Serial.println();

//       Serial.print("LM: ");
//       Serial.print(L_speed);
//       Serial.print(" / ");
//       Serial.println(L_aim);

//       Serial.print("RM: ");
//       Serial.print(R_speed);
//       Serial.print(" / ");
//       Serial.println(R_aim);

//       Serial.print("\nMAX: ");
//       Serial.print(Limit_speed);
//       Serial.print(" / ");
//       Serial.println(MAX_SPEED);
//     }
//     else
//     {
//       Serial.print("Time: ");
//       Serial.print(track_time/1000);
//       Serial.print(".");
//       Serial.println(track_time%1000);
//     }
//   }
// }

void display()
{
  if(millis() - display_timer >= 250) // every 250ms
  {
    display_timer = millis();

    Serial.println("CLEAR");

    if(activated)
    {
      if(random()%10 == 0)
      {
        Serial.println("I <3 Daisy! ");
      }
      else if(block_num == 0)
      {
        track_time = 0;
        Serial.println("Waiting for the starting line...");
      }
      else if(block_num == 1)
      {
        track_time = millis()-track_timer;
        Serial.println("Going as fast as possible!!!");
      }
      else
      {
        Serial.println("Finished the race.");
      }
    }
    else
    {
      Serial.println("Push the BTN3! whatcha w8ing 4?");
    }

    

    Serial.println(L_speed);
    Serial.println(L_aim);

    Serial.println(R_speed);
    Serial.println(R_aim);

    Serial.println(track_time);

    Serial.println(uint8_t(Sensor_active[0] + (Sensor_active[1]<<1) + (Sensor_active[2]<<2) + (Sensor_active[3]<<3) + (Sensor_active[4]<<4) + (Sensor_active[5]<<5) + (Sensor_active[6]<<6) + (Sensor_active[7]<<7)));
  }
}

// void display()
// {
//   if(millis() - display_timer >= 250) // every 250ms
//   {
//     display_timer = millis();

//     Serial.println(  "████████████████████████████████████████████");
//     Serial.print(    "█ STATUS: ");

//     if(block_num == 0)
//     {
//       Serial.println(          "Waiting for the starting line... █");
//     }
//     else if(block_num == 1)
//     {
//       Serial.println(          "Going as fast as possible!!!     █");
//     }
//     else
//     {
//       Serial.println(          "Finished the race.               █");
//     }

//     Serial.println(  "████████████████████████████████████████████");
//     Serial.print(    "█  Left motor: ");
//     for(int i = 0; i < 25; i++)
//       if(L_speed > i*10)
//         Serial.print("▒");
//       else
//         Serial.print("░");
//     Serial.println("   █");
//     Serial.print(    "█       (Aim): ");
//     for(int i = 0; i < 25; i++)
//       if(L_aim > i*10)
//         Serial.print("▒");
//       else
//         Serial.print("░");
//     Serial.println("   █");
//     Serial.println(  "████████████████████████████████████████████");
//     Serial.print(    "█ Right motor: ");
//     for(int i = 0; i < 25; i++)
//       if(R_speed > i*10)
//         Serial.print("▒");
//       else
//         Serial.print("░");
//     Serial.println("   █");
//     Serial.print(    "█       (Aim): ");
//     for(int i = 0; i < 25; i++)
//       if(R_aim > i*10)
//         Serial.print("▒");
//       else
//         Serial.print("░");
//     Serial.println("   █");
//   }
// }

void read_sensors()
{
  if(millis() - sensor_timer >= 50) // every 50ms
  {
    sensor_timer = millis();

    for(int i = 0; i <= 7; i++) // 8 sensors
    {
      int sensor_value = analogRead(Sensors[i]); // read current sensor

      if(Sensor_active[i]) // was black
      {
        if(sensor_value <= BLACK_TO_WHITE) // switch
          Sensor_active[i] = false;
      }
      else // was white
      {
        if(sensor_value >= WHITE_TO_BLACK) // switch
          Sensor_active[i] = true;
      }
    }

    
  }
}

void echo_sensors()
{
  if(millis() - echo_timer >= 200) // every 200ms
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long int duration = pulseIn(ECHO_PIN, HIGH, 5000); // , 5000
    Distance = (343*(duration/2))/1000;

    Limit_speed = MAX_SPEED; 

    if(Distance != 0)
    {
      if(Distance <= 80)
        Limit_speed = 0;
      else if(Distance <= 250)
        Limit_speed = Distance;   
    }
  }
}