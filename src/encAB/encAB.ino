
#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <sensor_msgs/JointState.h>
#include <TimerOne.h>

#define uno
#ifdef uno
  // pid 4, 0.007 to begin with 
  // right
  #define ENCODER_PINA1 3                       // Quadrature encoder A pin for right wheel  
  #define ENA 5 // brown
  #define IN1 6 // red
  #define IN2 7 // orange

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

// #define N   1612.0 // ticks per revolution, this is test by manually turn the motor 360 degree to get tick number.
// const float N = 20;
// const float N = 806;
ros::NodeHandle  nh;

double vl, vr;
volatile int right_ticks, left_ticks;
float temp[2]={0};
// rospy_tutorials::Floats lstate, rstate;
sensor_msgs::JointState jnt_state;

void set_angle_cb( const rospy_tutorials::Floats& cmd_msg) {
  vr = cmd_msg.data[0];
  vl = cmd_msg.data[1];
}

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", set_angle_cb);

ros::Publisher joint1_pub("/joint1_states_from_arduino", &jnt_state);
// ros::Publisher joint2_pub("/joint2_states_from_arduino", &rstate);

// volatile float pos[1] = {0}, vel[1] = {0};
// has to be double and cannot be volatile otherwise publish won't work (not knowing why)
float pos[2]={0}, vel[2]={0};

void setup() {
  // Serial.begin(57600);
  // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(joint1_pub);
  // nh.advertise(joint2_pub);

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

  Timer1.initialize(200000);  // 200ms
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), rEncoder, FALLING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), lEncoder, FALLING);

  Timer1.attachInterrupt(ISR_timerone);
  // TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // analogWrite(EnA, 80);  // left
}

void loop() {
  rpwmOut(constrain(vr, -249, 249));
  lpwmOut(constrain(vl, -249, 249));
  
  // sprintf(buffer, "degree： %d", encoderPos);
  // nh.loginfo(buffer);
  /*
    if ((now - lasttimepub) >= 200)
    {
      sprintf(buffer, "degree： %1d", vel[0]);
      nh.loginfo(buffer);
      // pos[0] = encoderPos * 0.447;
      // vel[0] = pos[0] * 10;
      // encoderPos = 0;
      // joint_name[0]='joint1';
      // must give length, otherwise gets runtime index error
      joint_state.position_length = 1;
      joint_state.velocity_length = 1;
      joint_state.position = pos;
      joint_state.velocity = vel;
      pub.publish(&joint_state);
      lasttimepub = now;
    }
  */
  // nh.spinOnce();
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
  /*
     if (digitalRead(ENCODER_PINB1) == digitalRead(ENCODER_PINA1) )
     {
      encoderPos++; //you may need to redefine positive and negative directions
     }
     else
     {
      encoderPos--;
     }
  */
}

void ISR_timerone() {
  cli();
  // 360 * encoderPos /N
  #ifdef uno
    temp[0] = right_ticks * 18.0;
    temp[1] = left_ticks * 18.0;
  #else
    temp[0] = right_ticks * 0.4467;
    temp[1] = left_ticks * 0.4467;
  #endif
  right_ticks = left_ticks = 0;
  sei();
  /*
    vel[0] = temp * 5.0;
    pos[0] += temp ;
    pos[0] = fmod(pos[0], 360.0);
  */
  vel[0] = temp[0] * 5.0;
  pos[0] += temp[0];
  
  vel[1] = temp[1] * 5.0;
  pos[1]+= temp[1];
  // let h/w interface do the normalization
  // pos = fmod(pos, 360.0);

  jnt_state.position_length=2;
  jnt_state.velocity_length=2;
  jnt_state.position=pos;   
  jnt_state.velocity=vel;
  joint1_pub.publish(&jnt_state);
  
  /*
  rstate.data_length = 2;
  rstate.data[0] = pos[0];
  rstate.data[1] = vel[0];  
  // joint1_pub.publish(&rstate);
  
  // need to find a way to pub left and right states in one pub
  lstate.data_length = 2;
  lstate.data[0] = pos[1];
  lstate.data[1] = vel[1];  
  joint2_pub.publish(&lstate);
  */
  
  nh.spinOnce();
}

void rpwmOut(float out) {
  if (out < 0) {
    // drive motor CW
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(out));

  }
  else {
    // drive motor CCW
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, out);

  }
}

void lpwmOut(float out) {
  if (out < 0) {
    // drive motor CW
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(out));

  }
  else {
    // drive motor CCW
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, out);

  }
}
