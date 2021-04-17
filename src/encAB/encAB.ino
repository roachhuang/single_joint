
//Processing incoming serial data
// #include <Messenger.h>
//Contain definition of maximum limits of various data type
// #include <limits.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Messenger object
// Messenger Messenger_Handler = Messenger();

#define RESET_PIN 12
#include <ros.h>
// #include <rospy_tutorials/Floats.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <TimerOne.h>

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

// #include <ArduinoHardware.h>
ros::NodeHandle_<ArduinoHardware, 2, 2, 128, 128> nh;

// pwm cmd
// int32_t vl, vr;
float vr, vl;

volatile int32_t right_ticks, left_ticks;

//void get_vr_cb(const std_msgs::Float32& cmd_msg) {
//  vr = cmd_msg.data;
//}

// void get_motor_cmd_cb(const rospy_tutorials::Floats &cmd_msg) {
void get_motor_cmd_cb(const std_msgs::Float32MultiArray &cmd_msg) {
  vr = cmd_msg.data[0];
  vl = cmd_msg.data[1];
  rpwmOut(vr);
  lpwmOut(vl);
}

ros::Subscriber<std_msgs::Float32MultiArray> motor_cmd_sub("motor_cmd", &get_motor_cmd_cb);
// ros::Subscriber<std_msgs::Float32> vl_sub("vl", &get_vl_cb);

std_msgs::Int32 int_ticksLeft;
std_msgs::Int32 int_ticksRight;
// ros::Publisher left_ticks_pub("lwheel", &int_ticksLeft);
// ros::Publisher right_ticks_pub("rwheel", &int_ticksRight);

// pub joint_state
sensor_msgs::JointState jnt_state;
ros::Publisher joints_pub("/jnt_frm_arduino", &jnt_state);
float temp[2]={0};
float pos[2]={0}, vel[2]={0};

void setup() {
  setup_motors();
  setup_encoders();

  nh.initNode();
  // nh.loginfo("roachbot wheel encoders:");
  // nh.subscribe(vl_sub);
  nh.subscribe(motor_cmd_sub);
  nh.advertise(joints_pub);
  
  //nh.advertise(left_ticks_pub);
  //nh.advertise(right_ticks_pub);
  
  right_ticks = left_ticks = vl = vr = 0;  
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

  Timer1.initialize(200000); // 200ms
  Timer1.attachInterrupt(isrTimerOne);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), lEncoder, RISING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), rEncoder, RISING);
  // TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
}

void loop() {
  //nh.spinOnce();

  //rpwmOut(vr);
  //lpwmOut(vl);

  //cli();
  //int_ticksLeft.data = left_ticks;
  //int_ticksRight.data = right_ticks;
  //sei();

  // left_ticks_pub.publish(&int_ticksLeft);
  // nh.spinOnce();
  // right_ticks_pub.publish(&int_ticksRight);
  nh.spinOnce();
  // cannot higher than 3 in my experiment, otherwise pid won't work
  // delay(3); // pub at 100hz (10ms)
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

void isrTimerOne() {

  /*
  temp[0] = right_ticks * 0.4467;
  temp[1] = left_ticks * 0.4467;
  */
  
  /*
    vel[0] = temp * 5.0;
    pos[0] += temp ;
    pos[0] = fmod(pos[0], 360.0);
  */
  
  vel[0] = float(right_ticks/901.0);  // temp[0] * 5.0;
  // pos[0] += temp[0];
  vel[1] = float(left_ticks/901.0);  // temp[1] * 5.0;
  right_ticks = left_ticks = 0;
  
  // pos[1]+= temp[1];
  
  // let h/w interface do the normalization
  // pos = fmod(pos, 360.0);

  // actuator_state.position = pos;
  // actuator.velocity = vel;

  // jnt_state.name.resize(2);
  // jnt_state.position.resize(2);
  // jnt_state.velocity.resize(2);
  // jnt_state.name[0]="rWheel";  
  // jnt_state.name[1]="lWheel";  
  // jnt_state.header.stamp = rs::Time::now();
  jnt_state.position_length=2;
  jnt_state.velocity_length=2;
  jnt_state.position=pos;   
  jnt_state.velocity=vel;
  joints_pub.publish(&jnt_state);  
  
  nh.spinOnce();
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
