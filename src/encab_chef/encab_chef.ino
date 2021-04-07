
//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"

//Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Ultrasonic sensor enable bit

bool ultrasonic_set = true;
bool blinkState;
//Ultrasonic sensor interfacing module

#include <NewPing.h>

#define TRIGGER_PIN  46  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     42  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Messenger object
Messenger Messenger_Handler = Messenger();
#define RESET_PIN 12

//This is PIN2 of Arduino Mega
#define MPU_6050_INT 0
#define LED_PIN 13
#define OUTPUT_READABLE_YAWPITCHROLL

//Ultrasonic pins definition
const int echo = 9, Trig = 10;
long duration, cm;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Battery level monitor for future upgrade
#define BATTERY_SENSE_PIN PC_4

float battery_level = 12;

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;    
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

// #define uno
#ifdef uno
// pid 4, 0.007 to begin with
// left
#define ENCODER_PINA1 3                       // Quadrature encoder A pin for right wheel
#define ENA 5 // brown
#define IN1 6 // red
#define IN2 7 // orange
// right
#define ENCODER_PINA2  2
#define ENB 11  // black
#define IN3 12  // yellow
#define IN4 13  // green
#define N 20
// MEGA 2560
/*
     JGB37-520
     red: motor+
     black: encoder power-
     yellow: signal line (motor pulse 11 pulses)
     green: signal line (resolution 11* reductiion ratio(90)=resolution)
     blue: encoder power+ (3.3v or 5v)
     white: motor power-
*/

#else
#define ENCODER_PINA1 2                       // Quadrature encoder A pin
#define ENCODER_PINB1 5                       // Quadrature encoder B pin
#define ENA 6
#define IN1 7 // left motor
#define IN2 8

#define ENCODER_PINA2  3
#define ENCODER_PINB2  4
#define ENB 9
#define IN3 10 // right motor
#define IN4 11
#define N 990
#endif

// ticks per revolution, this has to be tested by manually turn the motor 360 degree to get tick number.

//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;

volatile int32_t right_ticks, left_ticks;

/* 
void get_motor_cmd_cb(const std_msgs::Float32MultiArray &cmd_msg) {
  vr = cmd_msg.data[0];
  vl = cmd_msg.data[1];
  rpwmOut(vr);
  lpwmOut(vl);
}
*/

void Read_From_Serial()
{
  while (Serial.available() > 0)
  {
    int data = Serial.read();
    Messenger_Handler.process(data);
  }
}

void OnMessageCompleted()
{
  char reset[] = "r";
  char set_speed[] = "s";

  if (Messenger_Handler.checkString(reset))
  {
    Serial.println("Reset Done");
    Reset();
  }
  if (Messenger_Handler.checkString(set_speed))
  {
    //This will set the speed
    Set_Speed();
  }
}

//Setup Message Handler
void Setup_Messenger()
{
  pinMode(RESET_PIN, OUTPUT);
  //Set up Messenger
}

//Reset function
void Reset()
{
  // Reset_Encoder();
  // digitalWrite(GREEN_LED, HIGH);
  delay(1000);
  digitalWrite(RESET_PIN, LOW);
  // digitalWrite(GREEN_LED, LOW);
}

void SetupReset()
{
  // pinMode(GREEN_LED, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
  digitalWrite(RESET_PIN, HIGH);
}

//Set speed
void Set_Speed()
{
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
}

//Will update both motors
void Update_Motors()
{
  lpwmOut(motor_left_speed);
  rpwmOut(motor_right_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);
  Serial.print("\n");
}

void Update_Encoders()
{
  Serial.print("e");
  Serial.print("\t");
  Serial.print(left_ticks);
  Serial.print("\t");
  Serial.print(right_ticks);
  Serial.print("\n");
}

//Update battery function
void Update_Battery()
{
  //battery_level = analogRead(PC_4);

  Serial.print("b");
  Serial.print("\t");
  //Serial.print(battery_level);
  Serial.print("\n");
}

//Update time function
void Update_Time()
{
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
  {
    MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
  }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
}

void Update_all()
{
  //This will read from the index serial port
  Read_From_Serial();
  //Send time information through serial port
  // Update_Time();
  //Update motor values with corresponding speed and send speed values through serial port
  Update_Motors();
  Update_Encoders();
  //Update_Ultrasonic();
  //Send battery values through serial port
  //Update_Battery();
}

void setup() {
  Serial.begin(115200);

  setup_motors();
  setup_encoders();
  //Setup Reset pins
  SetupReset();
  Messenger_Handler.attach(OnMessageCompleted);  
}

void setup_motors() {
  // left
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // right
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void setup_encoders() {
  pinMode(ENCODER_PINA1, INPUT_PULLUP);                  // quadrature encoder input A
#ifndef uno
  pinMode(ENCODER_PINB1, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(ENCODER_PINB2, INPUT_PULLUP);                  // quadrature encoder input B
#endif
  pinMode(ENCODER_PINA2, INPUT_PULLUP);                  // quadrature encoder2 input A

  // Timer1.initialize(10000); // 200ms
  // Timer1.attachInterrupt(isrTimerOne);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), lEncoder, RISING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), rEncoder, RISING);
  // TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
}

void loop() {
  Update_all();

  //rpwmOut(vr);
  //lpwmOut(vl);
}

void lEncoder()  {
#ifdef uno
  left_ticks++;
#else
  left_ticks -= digitalRead(ENCODER_PINB1) ? -1 : +1;
#endif
}

void rEncoder()  {
#ifdef uno
  right_ticks++;
#else
  // CW->-1; CCW->+1
  right_ticks += digitalRead(ENCODER_PINB2) ? -1 : +1;
#endif
}

/*
  void isrTimerOne(){
  nh.spinOnce();
  //left_ticks_pub.publish(&int_ticksLeft);
  //right_ticks_pub.publish(&int_ticksRight);
  }
*/

void lpwmOut(float out) {
  // drive motor CW
  digitalWrite(IN1, out < 0);
  digitalWrite(IN2, out > 0);
  analogWrite(ENA, abs(out));
}

void rpwmOut(float out) {
  // drive motor CW
  digitalWrite(IN3, out < 0);
  digitalWrite(IN4, out > 0);
  analogWrite(ENB, abs(out));
}
