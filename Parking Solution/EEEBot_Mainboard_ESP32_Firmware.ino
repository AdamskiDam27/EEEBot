//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  EEEBot Firmware Code for the Mainboard ESP32        *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT
// read through the accompanying readme file and skeleton master code which is compatible with the firmware - only then, if you are still unsure, ask for help

// DO NOT modify or edit any of this code - for Project Week 3, the mainboard ESP32 code is provided for you, hence the term 'firmware'
// the only exception is to modify the HIGH/LOW states for the motors (if the motors do not spin the correct way) and the pin numbers for the encoders (if the encoder(s) do not increment/decrement as desired)

#include <Wire.h>
#include <ESP32Encoder.h>

// create two encoder variable types
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// create two signed encoder count variables of 16-bit size
int16_t enc1Count = 0;
int16_t enc2Count = 0;

#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

// L298 motor driver pin definitions
#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// setting up the PWM properties used for the motors
const int freq = 2000;
const int ledChannela = 0;  // assumed as the channel used for the left motor
const int ledChannelb = 1;  // assumed as the channel used for the righteft motor
const int resolution = 8; // 8-bit PWM signal

int servoPin = 13;  //the servo is attached to IO_13 on the ESP32

// setting up the PWM properties of the servo
// as an aside, the control signal of the SG90 servo is a PWM signal with a period of 20ms (50Hz) and the pulse duration has to be between 1ms to 2ms i.e. 5% duty cycle for the minimum angle of 0, and 10% duty cycle for the maximum angle of 180
// it is not recommended that you change any of the four values below
int dutyCycle = 5;
const int servoFrequency = 50;  // 50Hz signal
const int servoChannel = 2;     // channels 0 and 1 are used for the two motors on your EEEBot
const int servoResolution = 12;  // 12-bit PWM signal


void setup() {
  // enable the weak pull up resistors for the two encoders
  ESP32Encoder::useInternalWeakPullResistors=UP;

  // attach the relevant pins to each encoder
  encoder1.attachHalfQuad(34, 35);
  encoder2.attachHalfQuad(36, 39);

  // set the count of both encoders to 0
  encoder1.setCount(0);
  encoder2.setCount(0);

  // configure the motor direction control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  // pinMode(enA, OUTPUT);
  // pinMode(enB, OUTPUT);  // if defining some pins as PWM, do not set them as OUTPUT!

  // configure LED PWM functionalities
  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution); //servo setup on PWM channel 2, 50Hz, 12-bit (0-4095)

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
  Wire.onReceive(onReceive);    // receive event
  Wire.onRequest(onRequest);    // request event
  
  Serial.begin(115200);             // start serial for the output
  Serial.println("ESP32 Running");  // sanity check
}

void loop() {
  // continuously 'get' the value from each encoder
  enc1Count = encoder1.getCount();
  enc2Count = encoder2.getCount();

  // can be uncommented for checking what values are returned
  Serial.print(enc1Count);
  Serial.print("\t"); // print a 'tab' space between values
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
  /*
  Serial.print("Left Motor: ");
  Serial.print(leftMotor_speed);
  Serial.print("\t");
  Serial.print("Right Motor: ");
  Serial.print(rightMotor_speed);
  Serial.print("\t");
  Serial.print("Servo: ");
  Serial.println(servoAngle);
*/
  setSteeringAngle(servoAngle);
  runMotors(leftMotor_speed, rightMotor_speed);
}


// function to clear the I2C buffer
void emptyBuffer(void){
  Serial.println("Error: I2C Byte Size Mismatch");
  while(Wire.available())
  {
    Wire.read();
  }
}


// function to set the steering angle
void setSteeringAngle(int servoAngle){
  //
  dutyCycle = map((constrain(servoAngle, 0, 180)), 0, 180, 205, 410); // contrain() limits the minimum and maximum values to 0 and 180 respectively, map() proportionally scales values between 0 and 180 to values between 205 (5% duty cycle) and 410 (10% duty cycle)
  ledcWrite(servoChannel, dutyCycle); // write the control signal to the PWM
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
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);    
  }
}
