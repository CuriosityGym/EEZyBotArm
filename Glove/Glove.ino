#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


int indexFinger = A0;
int secondFinger = A1;
uint8_t rotatePercent=0, updownPercent=0, BackForwPercent=0, GripPercent=0;
uint8_t lastRotatePercent=0, lastUpdownPercent=0, lastBackForwPercent=0, lastGripPercent=0;

int rollMax=45, rollMin=-45;
int pitchMax=50, pitchMin=-50;
int BendSensorMax=680, BendSensorMin=570;
boolean debug=false;
int delayLevel=350;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  //Serial port initialization
  Serial.begin(115200);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();


  devStatus = mpu.dmpInitialize();
  // mpu stability (hold down the hand on flat surface for 25sec until mpu calibrate it.)
  //delay(25000);
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready

    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection

    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //    Serial.print(F("DMP Initialization failed (code "));
    //    Serial.print(devStatus);
    //   Serial.println(F(")"));
  }

}

void loop()
{
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {

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
    //  Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  
  int yaw=ypr[0] * 180 / M_PI;
  int pitch=ypr[1] * 180 / M_PI;
  int roll=ypr[2] * 180 / M_PI; 
  int frontBack = analogRead(secondFinger);
  int gripperBendSensor = analogRead(indexFinger);



    
       if(!debug)
      {       
           Serial.write(0xFF);//Start of Packet
      }
      else
      {
           Serial.println("------");
           Serial.println(0xFF,DEC);
      }
      
      roll=constrain(roll,rollMin,rollMax);        
      rotatePercent = map(roll, rollMin , rollMax, 0 , 100 );   
      if(!debug)
      {       
        Serial.write(rotatePercent);
      }
      else
      {
          Serial.println(rotatePercent,DEC);
      }
      
      
           
      pitch=constrain(pitch,pitchMin,pitchMax);      
      updownPercent=map(pitch, pitchMax , pitchMin , 0 , 100 );
     

       if(!debug)
      {       
         Serial.write(updownPercent);
      }
      else
      {
          Serial.println(updownPercent,DEC);
      }
     
        
      
      
      frontBack=constrain(frontBack,BendSensorMin,BendSensorMax);      
      BackForwPercent= map(frontBack, BendSensorMax,BendSensorMin, 0,100);
      if(!debug)
      {       
          Serial.write(BackForwPercent);
      }
      else
      {
           Serial.println(BackForwPercent,DEC);
      }
     
        
      
      
      
      
      gripperBendSensor=constrain(gripperBendSensor,BendSensorMin,BendSensorMax);        
      GripPercent= map(gripperBendSensor, BendSensorMax, BendSensorMin, 0, 100);
      if(!debug)
      {       
           Serial.write(GripPercent);
      }
      else
      {
           Serial.println(GripPercent,DEC);
      }
          
      
      if(!debug)
      {       
           Serial.write(0xFF);//End of Packet
      }
      else
      {
           Serial.println(0xFF,DEC);
      }
      
        

      
     
      
      /*Serial.print("Yaw:");
      Serial.println(yaw, DEC);
      
      Serial.print("Pitch:");
      Serial.println(pitch, DEC);
      
      Serial.print("Roll:");
      Serial.println(roll, DEC);*/
      
      delay(delayLevel);
#endif
  }
}

