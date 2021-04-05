/*

  Arduino code to control a differential drive robot via ROS Twist messages using a NodeMcu or ESP8266
  If you have questions or improvements email me at reinhard.sprung@gmail.com

  Launch a ros serial server to connect to:
    roslaunch rosserial_server socket.launch

  Launch a teleop gamepad node:
    roslaunch teleop_twist_joy teleop.launch joy_config:="insert gamepad type"


  MIT License

  Copyright (c) 2018 Reinhard Sprung

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/* Warning: it tooks me forever to pick the right pins on NodeMcu for roachbot. don't even try to change any of them unless
 *  you have pretty confidence in it.
 *  
 */// left motor
#define L_ENCODER_A D7   // gpio0 (d3)
#define L_ENCODER_B D0   // gpio16, Quadrature encoder B pin
// motor A
#define L_PWM 5  // D1
#define L_DIR 0  // D3

// motor B
#define R_PWM 4  // D2
#define R_DIR 2  // D4

// not that it has to use these pins. otherwsie, won't boot. don't change pins

// (GPIOs 4, 12, 14(d5), 15(d8) have hardware PWM, and the others are by software)
// right motor
#define R_ENCODER_A D5   // gpio05 (d1)
#define R_ENCODER_B D6   // gpio04 (d2)
#define N 990

// #include <math.h>
#include <ESP8266WiFi.h>

#ifndef STASSID
#define STASSID "c1225"
#define STAPSK  "c0918321359"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#define LED_BUILTIN 2 // Remapping the built-in LED since the NodeMcu apparently uses a different one.

// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
#define PWM_MIN 300

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void get_motor_cmd_cb(const std_msgs::Float32MultiArray &cmd_msg);
void lpwmOut(float out);
void rpwmOut(float out);
void ICACHE_RAM_ATTR lEncoder();
void ICACHE_RAM_ATTR rEncoder();
float mapPwm(float x, float out_min, float out_max);

// ROS serial server
IPAddress server(192, 168, 1, 112);
//ros::NodeHandle node;
ros::NodeHandle_<ArduinoHardware, 2, 2, 128, 128> node;

ros::Subscriber<std_msgs::Float32MultiArray> motor_cmd_sub("motor_cmd", &get_motor_cmd_cb);
std_msgs::Int32 int_ticksLeft;
std_msgs::Int32 int_ticksRight;
ros::Publisher left_ticks_pub("lwheel", &int_ticksLeft);
ros::Publisher right_ticks_pub("rwheel", &int_ticksRight);

bool _connected = false;
float vr, vl;

volatile int32_t right_ticks, left_ticks;

void get_motor_cmd_cb(const std_msgs::Float32MultiArray &cmd_msg) {
  vr = cmd_msg.data[0];
  vl = cmd_msg.data[1];
  rpwmOut(vr);
  lpwmOut(vl);
}

void setup()
{
  right_ticks = left_ticks = vl = vr = 0;
  setupPins();
  setupSerial();
  setupWiFi();   

  // Connect to rosserial socket server and init node. (Using default port of 11411)
  Serial.printf("Connecting to ROS serial server at %s\n", server.toString().c_str());
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(motor_cmd_sub);
  node.advertise(left_ticks_pub);
  node.advertise(right_ticks_pub);
}

void setupPins()
{
  // Status LED
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);

  pinMode(L_ENCODER_A, INPUT);
  //pinMode(L_ENCODER_B, INPUT);
  
  pinMode(R_ENCODER_A, INPUT);
  //pinMode(R_ENCODER_B, INPUT);

  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);  

  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), lEncoder, RISING);               // update encoder position 
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), rEncoder, RISING);
  stop();
}

void setupSerial()
{
  Serial.begin(115200);
  Serial.println();
}

void setupWiFi()
{
  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void stop()
{
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void loop()
{
  if (!rosConnected()) {
    Serial.println("oop! ros disconnected");
    stop();
  } else {
    cli();
    int_ticksLeft.data = left_ticks;
    int_ticksRight.data = right_ticks;
    sei();
    left_ticks_pub.publish(&int_ticksLeft);
    // right_ticks_pub.publish(&int_ticksRight);
  }
  node.spinOnce();
  delay(1);
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

void ICACHE_RAM_ATTR lEncoder() {
  //left_ticks++;
  left_ticks -= digitalRead(L_ENCODER_B) ? -1 : +1;
}

void ICACHE_RAM_ATTR rEncoder()  {
  // CW->-1; CCW->+1
  //right_ticks++;
  right_ticks += digitalRead(R_ENCODER_B) ? -1 : +1;
}

void lpwmOut(float out) {
  // drive motor CW
  digitalWrite(L_DIR, out > 0); 
  analogWrite(L_PWM, abs(out));
}

void rpwmOut(float out) {
  // drive motor CW
  digitalWrite(R_DIR, out > 0);  
  analogWrite(R_PWM, abs(out));
}
