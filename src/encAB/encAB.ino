
#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <sensor_msgs/JointState.h>
#include <TimerOne.h>

#define encoderPinA1      2                       // Quadrature encoder A pin
#define encoderPinB1      5                       // Quadrature encoder B pin
#define EnA 6
#define In1 7 // right motor
#define In2 8

#define encoderPinA2  3
#define EnB 9
#define In3 10 // left motor
#define In4 11


// #define N   1612.0 // ticks per revolution, this is test by manually turn the motor 360 degree to get tick number.
const float N = 806;
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

  pinMode(encoderPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoderPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(encoderPinA2, INPUT_PULLUP);                  // quadrature encoder2 input A

  //initialize Pin States
  // digitalWrite(encoderPinA1, LOW);
  // digitalWrite(encoderPinB1, LOW);
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  right_ticks = left_ticks = vl = vr = 0;

  Timer1.initialize(200000);  // 200ms
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), rEncoder, FALLING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), lEncoder, FALLING);

  Timer1.attachInterrupt(ISR_timerone);
  // TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
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
  left_ticks++;
}

void rEncoder()  {
  // CW->-1; CCW->+1
  right_ticks += digitalRead(encoderPinB1) ? 1 : -1;

  /*
     if (digitalRead(encoderPinB1) == digitalRead(encoderPinA1) )
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
  temp[0] = right_ticks * 0.4467;
  temp[1] = left_ticks * 0.4467;
  right_ticks = left_ticks  = 0;
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
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    analogWrite(EnA, abs(out));

  }
  else {
    // drive motor CCW
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    analogWrite(EnA, out);

  }
}

void lpwmOut(float out) {
  if (out < 0) {
    // drive motor CW
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
    analogWrite(EnB, abs(out));

  }
  else {
    // drive motor CCW
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
    analogWrite(EnB, out);

  }
}
