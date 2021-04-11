#define MPU_INT_PIN 18    // int#3
#define OUTPUT_READABLE_QUATERNION

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
// #include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//using namespace std;

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);

volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#include <NewPing.h>
#define TRIGGER_PIN  46  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     42  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Messenger object
Messenger Messenger_Handler = Messenger();

//Creating MPU6050 Object
//MPU6050 accelgyro(0x68);

#define LED_PIN 13

//Ultrasonic sensor enable bit
bool ultrasonic_set = true;
bool blinkState;

//Battery level monitor for future upgrade (these are not implemented)
// #define BATTERY_SENSE_PIN PC_4
// float battery_level = 12;
// Reset pin for resetting MEGA 2560. if this PIN set high, Tiva C will reset
// #define RESET_PIN 30

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
//////////////////////// IMU /////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];

// #define OUTPUT_READABLE_QUATERNION
//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];

void Setup_MPU6050()
{
  Wire.begin();
  // initialize device
  Serial.println("sets the acceleromenter to +/- 2g and the gyro to 250% per second.");
  accelgyro.initialize();

  // verify connection
  Serial.println("check that it can find the I2C device addr associated with the IMU. 0x68 or 0x69");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Initialize DMP in MPU 6050
  Setup_MPU6050_DMP();
}

void Setup_MPU6050_DMP()
{
  //DMP Initialization
  devStatus = accelgyro.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  /*
    accelgyro.setXGyroOffset(220);
    accelgyro.setXGyroOffset(76);
    accelgyro.setXGyroOffset(-85);
    accelgyro.setXGyroOffset(1788);
  */
  accelgyro.setXAccelOffset(-3796);
  accelgyro.setYAccelOffset(441);
  accelgyro.setZAccelOffset(573);
  accelgyro.setXGyroOffset(144);
  accelgyro.setYGyroOffset(58);
  accelgyro.setZGyroOffset(-10);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    accelgyro.setDMPEnabled(true);
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = accelgyro.getIntStatus();
  
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = accelgyro.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void Update_MPU6050()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  ///Update values from DMP for getting rotation vector
  Update_MPU6050_DMP();
}

//Update MPU6050 DMP functions
void Update_MPU6050_DMP()
{
  //DMP Processing
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    mpuInterrupt = true;
  }
  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();
  //get current FIFO count
  fifoCount = accelgyro.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount > 512) {
    // reset so we can continue cleanly
    accelgyro.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print("i"); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
    Serial.print(q.w);
    Serial.print("\n");
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
  }
}

//////////////////////////////////////////////////////////////////////

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
  //pinMode(RESET_PIN, OUTPUT);
  //Set up Messenger
}

//Reset function
void Reset()
{
  Reset_Encoder();
  delay(1000);
  // digitalWrite(RESET_PIN, LOW);
  delay(3000);
  // digitalWrite(RESET_PIN, HIGH);
}

void SetupReset()
{
  // pinMode(GREEN_LED, OUTPUT);
  //pinMode(RESET_PIN, OUTPUT);

  ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
  //digitalWrite(RESET_PIN, HIGH);
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

/////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  setup_motors();
  setup_encoders();
  // SetupUltrasonic();
  Setup_MPU6050();  
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

void Update_Ultra_Sonic()
{
  if (ultrasonic_set == true)
  {
    //delay(50);
    int distance = sonar.ping_cm();
    //Sending through serial port
    Serial.print("u");
    Serial.print("\t");
    Serial.print(distance);
    Serial.print("\n");
  }
}

void Reset_Encoder() {
  left_ticks = right_ticks = 0;
}

void setup_encoders() {
  pinMode(ENCODER_PINA1, INPUT_PULLUP);                  // quadrature encoder input A
#ifndef uno
  pinMode(ENCODER_PINB1, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(ENCODER_PINB2, INPUT_PULLUP);                  // quadrature encoder input B
#endif
  pinMode(ENCODER_PINA2, INPUT_PULLUP);                  // quadrature encoder2 input A

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), lEncoder, RISING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), rEncoder, RISING);
  // TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
}

void Update_all()
{
  //This will read from the index serial port
  Read_From_Serial();
  //Send time information through serial port
  //Update_Time();
  Update_Encoders();
  // Update_Ultrasonic();
  //Update motor values with corresponding speed and send speed values through serial port
  Update_Motors();
  Update_MPU6050();

  //Send battery values through serial port
  //Update_Battery();
}

void loop() {
  Update_all();
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
