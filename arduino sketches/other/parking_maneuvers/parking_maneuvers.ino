//**************************************************//
//EEEbot parking maneuvers                          //
//Author: Adam Fairweather                          //
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
byte leftSpeedSetting = 255;
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
//create a variable to hold the steering angle, between 5% and 10% duty = 204 - 408 on a 12 bit PWM
float steeringAngle = 298; //variable for servo position

int steeringFlag = 1;

long oldposition = 0;
long newposition;
int count = 0;
int zerocounter = 0;
int runtime = 15800;
int turnFlag;

long startTime;

void setup() {
  
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

  //getting the time
  startTime = millis();

  //initalise serial communication
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  turn180(servoChannel);
  //Serial.println(encoder.getCount());
  //goForwards();
  //setSpeed(255, 255);
}

void goForwards(){
  //car just goes forwards
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stop(){
  //stops the car from moving
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}

void goBackwards(){
  //makes the car reverse
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void setSpeed(int leftSpeed, int rightSpeed){
  //Set individual motor speed
  ledcWrite(ledChannela, leftSpeed);
  ledcWrite(ledChannelb, rightSpeed);
}

void turn180(int servoChannel){
  ledcWrite(servoChannel, 204);
  goForwards();
  setSpeed(170, 255);
  delay(2);
  Serial.println("Delay over");
  stop();
  setSpeed(0, 0);
}