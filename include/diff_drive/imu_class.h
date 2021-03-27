#ifndef IMU_CLASS_H_
define IMU_CLASS_H_
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <geometry_msgs/Vector3.h>

class ImuClass{
    public:
        ImuClass(ros::NodeHandle &nh);
        ~Imuclass();
        void dumpRead();
    private:
        //Creating a MPU6050 handle which can be used for MPU 9250
        MPU6050 mpu;
        //Creating ROS publisher object for IMU orientation
        ros::Publisher imu_pub("imu_data", &orient);
        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        //Creating orientation message header
        geometry_msgs::Vector3 orient;
        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

        void initPublishers();
        void init();        
        void imuIsr();       
}

ImuClass::ImuClass(ros::NodeHandle &nh){
    nh.advertise(imu_pub);
    init();
}

void ImuClass::init(){
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(19), ImuClass::imuIsr, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        ;
    }    
}

void ImuClass::dumpRead(){
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        ;
        nh.spinOnce();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x01) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //Assigning YAW,PITCH,ROLL to vector message and publishing the values

        orient.x = ypr[0] * 180 / M_PI;
        orient.y = ypr[1] * 180 / M_PI;
        orient.z = ypr[2] * 180 / M_PI;
        imu_pub.publish(&orient);
    }
}
void::imuClass imuIsr(){
    mpuInterrupt = true;    
}

#endif