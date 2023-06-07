/*

               Video tracking Software for Arduino


               
      by Gal Arbel
       2023
*/

#include <Arduino.h>
#include "clicli.h"
#include "PID.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Servo.h>


//Gyro things:
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//motors attach:
Servo FrontESC;
Servo BackESC;


PID::PID(int MotorPinFront, int MotorPinBack){
  //variable:
  _MotorPinFront = MotorPinFront;
  _MotorPinBack = MotorPinBack;
  //pinMode(x, OUTPUT); //pins
}

void PID::begin() {
  Serial.begin(115200);
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
    // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

    // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-1376); //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-37);
  mpu.setXAccelOffset(1788); // 1688 factory default for my test chip
  mpu.setYAccelOffset(1788); // 1688 factory default for my test chip
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);


    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));  
  }


  //myyy code
  FrontESC.attach(_MotorPinFront,  1000, 2000);
  BackESC.attach(_MotorPinBack,  1000, 2000);

}

void PID::MotorTest(int mot, int speed){
  if (mot == 0){ //Front
    FrontESC.write(speed);
  } else if (mot == 1) {
    BackESC.write(speed);
  }
}

void PID::getattached(int mot){
  if (mot == 0){ //Front
    Serial.println(FrontESC.attached());
  } else if (mot == 1) {
    Serial.println(BackESC.attached());
  }
  
}