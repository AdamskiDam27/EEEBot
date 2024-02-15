//**************************************************//
//University of Nottingham                          //
//EEEBOT                                            //
//Adam Fairweather                                  //      
//ASSUMPTION: Channel A is LEFT, Channel B is RIGHT //
//**************************************************//
#include <ESP32Encoder.h>

#define enA 33  //EnableA command line
#define enB 25  //EnableA command line

#define INa 26  //Channel A Direction
#define INb 27 //Channel A Direction
#define INc 14 //Channel B Direction
#define INd 12  //Channel B Direction

//defining the encoders
ESP32Encoder encoder;
ESP32Encoder encoder2;

//inital speed for each motor
byte leftSpeedSetting = 0;
byte rightSpeedSetting = 255;

//setting PWM properties
const int freq = 2000;
const int servoFrequency = 50;
const int ledChannela = 0;
const int ledChannelb = 1;
const int servoChannel = 2;
const int resolution = 8;
const int servoResolution = 12;

int servoPin = 13;
float steeringAngle = 408; //variable for servo position, between 204 and 408

int steeringFlag = 1;

int count = 0;

float distance = 0;



void setup(){

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  //configure LED PWM functionalities
  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution); //servo setup on PWM2, 50Hz, 12-bit (0-4096)

  //attach the channel to the GPIO to be controlled
  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  //setting up the encoder
  encoder.attachHalfQuad(34, 35); //set the pins
  encoder.setCount(37); //set the starting count value
  encoder.clearCount(); //clear encoder's raw count and set tracked count to zero
  Serial.println("Encoder Start = "+String((int32_t)encoder.getCount()));

  //initalise serial communication
  Serial.begin(9600);
}

void loop(){
  //where the code will continuously loop
  while (distance < 1000){
    //EEEBot begins to move
    goForwards();
    setSpeed(leftSpeedSetting, rightSpeedSetting);
  
    //check the encoder count
    count = encoder.getCount();
    Serial.println(count);

    //calculate the distance it has travelled
    distance = (count/46)*18;
  }
  stop();
}

void stop(){
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}

void setSpeed(int leftSpeed, int rightSpeed){
  //Set individual motor speed
  ledcWrite(ledChannela, leftSpeed);
  ledcWrite(ledChannelb, rightSpeed);
  //delay(25);
}

void goForwards(){
  //car will go forward indefinitly
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}