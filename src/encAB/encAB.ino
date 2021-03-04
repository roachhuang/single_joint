
#include <ros.h>
// #include <rospy_tutorials/Floats.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define uno
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
#else
#define ENCODER_PINA1 2                       // Quadrature encoder A pin
#define ENCODER_PINB1 5                       // Quadrature encoder B pin 
#define ENA 6
#define IN1 7 // right motor
#define IN2 8

#define ENCODER_PINA2  3
#define ENB 9
#define IN3 10 // left motor
#define IN4 11
#define N 806
#endif

// ticks per revolution, this has to be tested by manually turn the motor 360 degree to get tick number.

ros::NodeHandle nh;
// pwm cmd
// int32_t vl, vr;
float vr, vl;

volatile int32_t right_ticks, left_ticks;

void get_vr_cb(const std_msgs::Float32& cmd_msg) {
  vr = cmd_msg.data;
}

void get_vl_cb(const std_msgs::Float32& cmd_msg) {
  vl = cmd_msg.data;
}

ros::Subscriber<std_msgs::Float32> vr_sub("vr", &get_vr_cb);
ros::Subscriber<std_msgs::Float32> vl_sub("vl", &get_vl_cb);

std_msgs::Int32 int_ticksLeft;
std_msgs::Int32 int_ticksRight;
ros::Publisher left_ticks_pub("lwheel", &int_ticksLeft);
ros::Publisher right_ticks_pub("rwheel", &int_ticksRight);

void setup() {
  nh.initNode();
  // nh.loginfo("roachbot wheel encoders:");
  nh.subscribe(vl_sub);
  nh.subscribe(vr_sub);
  nh.advertise(left_ticks_pub);
  nh.advertise(right_ticks_pub);

  pinMode(ENCODER_PINA1, INPUT_PULLUP);                  // quadrature encoder input A
#ifndef uno
  pinMode(ENCODER_PINB1, INPUT_PULLUP);                  // quadrature encoder input B
#endif
  pinMode(ENCODER_PINA2, INPUT_PULLUP);                  // quadrature encoder2 input A

  //initialize Pin States
  // digitalWrite(ENCODER_PINA1, LOW);
  // digitalWrite(ENCODER_PINB1, LOW);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  right_ticks = left_ticks = vl = vr = 0;

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), lEncoder, FALLING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), rEncoder, FALLING);
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
}

void loop() {
  rpwmOut(vr);
  lpwmOut(vl);

  cli();
  int_ticksLeft.data = left_ticks;
  int_ticksRight.data = right_ticks;
  sei();

  left_ticks_pub.publish(&int_ticksLeft);
  right_ticks_pub.publish(&int_ticksRight);

  nh.spinOnce();
  delay(10);
}

void lEncoder()  {
#ifdef uno
  left_ticks++;
#else
  // right_ticks += digitalRead(ENCODER_PINB2) ? 1 : -1;
#endif
}

void rEncoder()  {
#ifdef uno
  right_ticks++;
#else
  // CW->-1; CCW->+1
  right_ticks += digitalRead(ENCODER_PINB1) ? 1 : -1;
#endif
}

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
