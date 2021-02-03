
#include <ros.h>
#include <rospy_tutorials/Floats.h>
// #include <sensor_msgs/JointState.h>
#include <TimerOne.h>

#define encoderPinA1      2                       // Quadrature encoder A pin
#define encoderPinB1      5                       // Quadrature encoder B pin
#define EnA 6
#define In1 7 // left
#define In2 8
// #define N   1612.0 // ticks per revolution, this is test by manually turn the motor 360 degree to get tick number.
const float N = 806;
ros::NodeHandle  nh;

double output = 0;
volatile int encoderPos = 0;
float temp = 0;
rospy_tutorials::Floats actuator_state;
// sensor_msgs::JointState joint_state;

void set_angle_cb( const rospy_tutorials::Floats& cmd_msg) { 
  output = cmd_msg.data[0];
}

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", set_angle_cb);
// ros::Publisher pub("joint_states_from_arduino", &joint_state);
ros::Publisher pub("/joint_states_from_arduino", &actuator_state);
// volatile float pos[1] = {0}, vel[1] = {0};
// has to be double and cannot be volatile otherwise publish won't work (not knowing why)
double pos = 0, vel = 0;

void setup() {
  // Serial.begin(57600);
  // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  pinMode(encoderPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoderPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  //initialize Pin States
  // digitalWrite(encoderPinA1, LOW);
  // digitalWrite(encoderPinB1, LOW);
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  Timer1.initialize(200000);  // 200ms
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), readEncoder, FALLING);               // update encoder position
  Timer1.attachInterrupt(ISR_timerone);
  // TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  // analogWrite(EnA, 80);  // left

  // joint_state.position_length = 1;
  // joint_state.velocity_length = 1;
 
}

void loop() {
  // pos[0] = (float)(encoderPos * 0.4467); // in degrees
  // now = millis();

  pwmOut(constrain(output, -249, 249));
  // Serial.println(pos[0]);

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

void readEncoder()  {
  // CW->-1; CCW->+1
  encoderPos += digitalRead(encoderPinB1) ? 1 : -1;

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
  temp = encoderPos * 0.4467;
  encoderPos = 0;
  sei();
  /*
    vel[0] = temp * 5.0;
    pos[0] += temp ;
    pos[0] = fmod(pos[0], 360.0);
  */
  vel = temp * 5.0;
  pos += temp ;
  pos = fmod(pos, 360.0);
  // actuator_state.position = pos;
  // actuator.velocity = vel;
  actuator_state.data_length = 2;
  actuator_state.data[0] = pos;
  actuator_state.data[1] = vel;
  pub.publish(&actuator_state);
  nh.spinOnce();
}

void pwmOut(float out) {
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
