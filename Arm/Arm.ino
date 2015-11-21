
//https://www.arduino.cc/en/Tutorial/ReadASCIIString
//https://www.arduino.cc/en/Reference/ParseInt
//http://microcontrollerkits.blogspot.in/2014/01/arduino-usb-control-servo-motor.html
//http://wiki.jmoon.co/sensors/hc-05-bluetooth/
//http://www.instructables.com/id/How-to-control-servo-motor-from-android-app/

#include <Servo.h> 
Servo rotationServo;  // rotation
Servo upDownServo;  //  updown
Servo frontBackServo;  //  BackForw
Servo gripperServo;  // grip

#define DATALENGTH 6

int pos1 = 0;    // variable to store the servo position
int pos2 = 0;    // variable to store the servo position
int pos3 = 0;    // variable to store the servo position
int pos4 = 0;    // variable to store the servo position

int rotationMax=180;
int rotationMin=0;


int upDownMax=100;
int upDownMin=40;

int frontBackMax=160;
int frontBackMin=40;

int gripperMax=70;
int gripperMin=20;

uint8_t rotationPercent=0;
uint8_t upDownPercent=0;
uint8_t frontBackPercent=0;
uint8_t gripperPercent=0;

boolean stringComplete=false;

boolean packetStarted=false;
boolean packetComplete=false;

int rotationServoAngle=90;
int upDownServoAngle=80;
int frontBackServoAngle=90;
int gripperServoAngle=90;
int dataCount=0;


void setup()
{
      Serial.begin(115200);
      pinMode(13, OUTPUT);
      delay(5000);
      rotationServo.attach(9);  // attaches the servo on pin 9 to the servo object 
      upDownServo.attach(10);  // attaches the servo on pin 10 to the servo object 
      frontBackServo.attach(11);  // attaches the servo on pin 11 to the servo object 
      gripperServo.attach(6);  // attaches the servo on pin 6 to the servo object 
      //initial State of MeArm      
      rotationServo.write(rotationServoAngle);            
      delay(100);  
      upDownServo.write(upDownServoAngle);  
      delay(100); 
      frontBackServo.write(frontBackServoAngle);  
      delay(100); 
      gripperServo.write(gripperServoAngle); 
      delay(100);
      

      //Serial.println("Ready");
  
  }

  
void loop()
{
          serialEvent();
          
          if(packetComplete)
          {

           /* Serial.println("rotationPercent"+rotationPercent);
            Serial.println("upDownPercent"+upDownPercent);
            Serial.println("frontBackPercent"+frontBackPercent);
            Serial.println("gripperPercent"+gripperPercent);*/
           
            rotationServo.write(rotationServoAngle);             
            delay(15);             
            upDownServo.write(upDownServoAngle);           
            delay(15);            
            gripperServo.write(gripperServoAngle); 
            delay(15);           
           
            frontBackServo.write(frontBackServoAngle);              
            delay(15);     
            
          }      
      
}


void serialEvent()
{
  
    if(Serial.available()> DATALENGTH-1)
    {
      packetComplete=false;
      uint8_t serialStream[6]={};  
      for(uint8_t dataCounter=0;dataCounter<DATALENGTH;dataCounter++)
      {
        serialStream[dataCounter]=(uint8_t)Serial.read();
      }  

     if(serialStream[0]==0xFF && serialStream[5]==0xFF)
     {
        packetComplete=true;
        rotationPercent=serialStream[1];
        upDownPercent=serialStream[2];
        frontBackPercent=serialStream[3];
        gripperPercent=serialStream[4]; 

       // Serial.print("rotationPercent: ");
        //////Serial.println(rotationPercent);
        
        
        //Serial.print("upDownPercent: ");        
        //Serial.println(upDownPercent);
        
        
        //Serial.print("frontBackPercent: ");        
        //Serial.println(frontBackPercent);
        
        //Serial.print("gripperPercent: ");  
        //Serial.println(gripperPercent); 

        rotationServoAngle=map(rotationPercent,0,100,rotationMin, rotationMax);
        upDownServoAngle=map(upDownPercent,0,100,upDownMin, upDownMax);
        frontBackServoAngle=map(frontBackPercent,0,100,frontBackMin, frontBackMax);
        gripperServoAngle=map(gripperPercent,0,100,gripperMin, gripperMax);

        //Serial.print("rotationServoAngle: ");  
        //Serial.println(rotationServoAngle); 

              
     }

     
    }
    

}

  
