//**************************************************//
//University of Nottingham                          //
//EEEBOT 13/12/2023                                 //
//Adam Fairweather                                  //      
//ASSUMPTION: Channel A is LEFT, Channel B is RIGHT //
//**************************************************//
#include <ESP32Encoder.h>
#include <Wire.h>

//create 2 encoder variables
ESP32Encoder encoder1;
ESP32Encoder encoder2;

//create two signed encoder count variables of 16-bit size
int16_t enc1Count = 0;
int16_t enc2Count = 0;

#define I2C_SLAVE_ADDR 0x04 //4 in heaxadecimal

#define enA 33  //EnableA command line
#define enB 25  //EnableA command line

#define INa 26  //Channel A Direction
#define INb 27 //Channel A Direction
#define INc 14 //Channel B Direction
#define INd 12  //Channel B Direction

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
int dutyCycle = 5;

int servoPin = 13;
float steeringAngle = 298; //variable for servo position

long startTime;

void setup() {
  //enable the weak pull up resistors for the two encoders
  ESP32Encoder::useInternalWeakPullResistors=UP;

  //attach the relevent pins to each encoder
  encoder1.attachHalfQuad(34, 35);
  encoder2.attachHalfQuad(36, 39);

  //set the count of both encoders to 0
  encoder1.setCount(0);
  encoder2.setCount(0);

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

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  //getting the time
  startTime = millis();

  //initalise serial communication
  Serial.begin(9600);
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
  //continously get the value from each encoder
  enc1Count = encoder1.getCount();
  enc2Count = encoder2.getCount();

  Serial.println(enc1Count);
  Serial.println(enc2Count);
  
}

// this function executes when data is requested from the master device
void onRequest(){
  // depending on the size of the encoder count value, you may need to make use of bits 32 to 17 to send larger values
  
  //Wire.write((byte)((enc1Count & 0xFF000000) >> 24)); // bits 32 to 25 of enc1Count
  //Wire.write((byte)((enc1Count & 0x00FF0000) >> 16)); // bits 24 to 17 of enc1Count
  Wire.write((byte)((enc1Count & 0x0000FF00) >> 8));    // first byte of enc1Count, containing bits 16 to 9
  Wire.write((byte)(enc1Count & 0x000000FF));           // second byte of enc1Count, containing the 8 LSB - bits 8 to 1

  //Wire.write((byte)((enc2Count & 0xFF000000) >> 24)); // bits 32 to 25 of enc2Count
  //Wire.write((byte)((enc2Count & 0x00FF0000) >> 16)); // bits 24 to 17 of enc2Count
  Wire.write((byte)((enc2Count & 0x0000FF00) >> 8));    // first byte of enc2Count, containing bits 16 to 9
  Wire.write((byte)(enc2Count & 0x000000FF));           // second byte of enc2Count, containing the 8 LSB - bits 8 to 1
}

// this function executes whenever data is received from the master device
void onReceive(int howMany){
  if(howMany != 6){  // for 3 16-bit numbers, the data will be 6 bytes long - anything else is an error
    emptyBuffer();
    return;
  }

  // set up variables for the three 16-bit values
  int16_t leftMotor_speed = 0;
  int16_t rightMotor_speed = 0;
  int16_t servoAngle = 0;
  
  uint8_t leftMotor_speed16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t leftMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)
  uint8_t rightMotor_speed16_9 = Wire.read(); // receive bits 16 to 9 of y (one byte)
  uint8_t rightMotor_speed8_1 = Wire.read();  // receive bits 8 to 1 of y (one byte)
  uint8_t servoAngle16_9 = Wire.read();       // receive bits 16 to 9 of z (one byte)
  uint8_t servoAngle8_1 = Wire.read();        // receive bits 8 to 1 of z (one byte)

  leftMotor_speed = (leftMotor_speed16_9 << 8) | leftMotor_speed8_1;    // combine the two bytes into a 16 bit number
  rightMotor_speed = (rightMotor_speed16_9 << 8) | rightMotor_speed8_1; // combine the two bytes into a 16 bit number
  servoAngle = (servoAngle16_9 << 8) | servoAngle8_1;                   // combine the two bytes into a 16 bit number

  // verify that the correct values are received via the serial monitor
  Serial.print("Left Motor: ");
  Serial.print(leftMotor_speed);
  Serial.print("\t");
  Serial.print("Right Motor: ");
  Serial.print(rightMotor_speed);
  Serial.print("\t");
  Serial.print("Servo: ");
  Serial.println(servoAngle);

  setSteeringAngle(servoAngle);
  runMotors(leftMotor_speed, rightMotor_speed);
}

// function to set the steering angle
void setSteeringAngle(int servoAngle){
  //
  dutyCycle = map((constrain(servoAngle, 0, 180)), 0, 180, 205, 410); // contrain() limits the minimum and maximum values to 0 and 180 respectively, map() proportionally scales values between 0 and 180 to values between 205 (5% duty cycle) and 410 (10% duty cycle)
  ledcWrite(servoChannel, dutyCycle); // write the control signal to the PWM
}

// function to clear the I2C buffer
void emptyBuffer(void){
  Serial.println("Error: I2C Byte Size Mismatch");
  while(Wire.available())
  {
    Wire.read();
  }
}

// function to run the motors - you may need to modify the HIGH/LOW states to get each wheel to rotate in the desired direction
void runMotors(int leftMotor_speed, int rightMotor_speed){
  // limit the speed value between -255 and 255 as the PWM value can only be between 0 and 255 - the negative is handled below
  leftMotor_speed = constrain(leftMotor_speed, -255, 255);
  rightMotor_speed = constrain(rightMotor_speed, -255, 255);

  // vary the motor speeds - use the absolute value to remove the negative
  ledcWrite(ledChannela, abs(leftMotor_speed));
  ledcWrite(ledChannelb, abs(rightMotor_speed));

  // if the speed value is negative, run the motor backwards
  if (leftMotor_speed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);    
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotor_speed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);    
  }
}