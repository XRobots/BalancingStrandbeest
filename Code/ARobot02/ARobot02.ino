#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

double Pk1 = 2000;  
double Ik1 = 5000;
double Dk1 = 120;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

RF24 radio(25, 24); // CE, CSN
const byte addresses[][6] = {"00003", "00004"};

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//ODrive Object
ODriveArduino odrive(Serial1);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO 

    int16_t RLR;
    int16_t RFB;
    int16_t LLR;
    int16_t LFB;

};

RECEIVE_DATA_STRUCTURE mydata_remote;

int requested_state;   

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 11, Teensy is 11, Teensy++ is 6)
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


float pitch;
long velocity;

float trimPot;
float trimAngle;

float RFB;
float RFBFiltered;
int RLR;

long drive1;
long drive2;

int switch1;
int switch2;

int arm1;
int arm2;

int arm1Filtered;
int arm2Filtered;

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timers
unsigned long count;

int loopTime;
int previousLooptime;

unsigned long previousSafetyMillis;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high



// ****************** SETUP ******************************

void setup() {


    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);

    radio.startListening();
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    //ODrive serial port
    Serial1.begin(115200);

    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(120);
    mpu.setYGyroOffset(-95);
    mpu.setZGyroOffset(-24);
    mpu.setXAccelOffset(-3506);
    mpu.setYAccelOffset(1304);
    mpu.setZAccelOffset(562);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

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

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-50000, 110000);  // walk backwards slower because it doesn't work well mechanically
    PID1.SetSampleTime(10);

    pinMode(A0, INPUT);
    pinMode(15, INPUT_PULLUP);    // ENABLE
    pinMode(16, INPUT_PULLUP);    // INIT

}

// ********************* MAIN LOOP *******************************

void loop() {  
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
              previousMillis = currentMillis;       
      
              loopTime = currentMillis - previousLooptime;
              previousLooptime = currentMillis;      
      
      
              if (radio.available()) {
                          radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                          previousSafetyMillis = currentMillis; 
              } 
      
             // check if remote has become disconnected
      
              if(currentMillis - previousSafetyMillis > 200) {         
                  Serial.println("*no data* ");
                  mydata_remote.RLR = 512;
                  mydata_remote.RFB = 512;
                  mydata_remote.LLR = 512;
                  mydata_remote.LFB = 512;
      
              } 
      
              switch1 = digitalRead(16);    // init
              switch2 = digitalRead(15);    // motor enable
      
              if (switch1 == 0) {     // init Odrive
                  Serial.println("INIT");
                  OdriveInit();
              }
           
      
              RFB = (float) (mydata_remote.RFB - 512) / 100;
              RLR = ((mydata_remote.RLR - 512)  * 100) + 3000;
      
             // print control data, count and mode to terminal       
              Serial.print("Data: ");
              Serial.print(pitch);
              Serial.print(" , ");
              Serial.print(Setpoint1);
              Serial.print(" , ");
              Serial.print(switch1);
              Serial.print(" , ");
              Serial.print(switch2);
              Serial.print(" , ");
              Serial.print(drive1);
              Serial.println();
              
      
              RFBFiltered = filter(RFB, RFBFiltered, 15);        
      
              trimPot = analogRead(A0);
              trimAngle = (trimPot/100)-5;          
      
                
              if (IMUdataReady == 1) {
                readAngles();
              }      
           
              pitch = (ypr[1] * 180/M_PI);             
      
      
              // other PID stuff
              Setpoint1 = trimAngle;
              Input1 = pitch;
              PID1.Compute();

              drive1 = Output1;
              drive2 = Output1*-1;
    
              if (switch2 == 0) {                         
                odrive.SetVelocity(0, drive1); 
                odrive.SetVelocity(1, drive2);  
                Serial.println("DRIVE");
              }
      
              else {
                odrive.SetVelocity(0, 0); 
                odrive.SetVelocity(1, 0);
              }     
                
      
      }     // end of timed loop         
   
}     // end  of main loop


// filter function

float filter(float lengthOrig, float currentValue, int filter) {
  float lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}



