//**************************************************//
//University of Nottingham                          //
//EEEBOT 13/12/2023                                 //
//Adam Fairweather                                  //      
//ASSUMPTION: Channel A is LEFT, Channel B is RIGHT //
//**************************************************//

#include <ESP32Encoder.h>
#include <Wire.h>

#define enA 33  //EnableA command line
#define enB 25  //EnableA command line

#define INa 26  //Channel A Direction
#define INb 27 //Channel A Direction
#define INc 14 //Channel B Direction
#define INd 12  //Channel B Direction

#define SERVO 2

#define I2C_SLAVE_ADDR 0x04

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
float steeringAngle = 298; //variable for servo position

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

  //getting the time
  startTime = millis();

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceive);

  //initalise serial communication
  Serial.begin(9600);
}

void onReceive(char direction){
  //what the slave does when it receives a signal
  switch (direction){
    case 'l':
      ledcWrite(SERVO, 204);
      break;
    case 'r':
      ledcWrite(SERVO, 408);
      break;
  }
}

void goForwards(){
  //car goes forwards indefinitly
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
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
}

void figureOfEight(int servoChannel){
  ledcWrite(servoChannel, 204);
  goForwards();
  setSpeed(255, 255);
  delay(5000);
  ledcWrite(servoChannel, 408);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  goForwards();
}
