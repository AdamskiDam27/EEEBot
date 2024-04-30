#include <Wire.h>
#include <ESP32Encoder.h>

// create two encoder variable types
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// create two signed encoder count variables of 16-bit size
int16_t enc1Count = 0;
int16_t enc2Count = 0;

#define I2C_SLAVE_ADDR 0x04 // I2C slave address

// L298 motor driver pin definitions
#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

const int freq = 2000;
const int ledChannela = 0;  // assumed as the channel used for the left motor
const int ledChannelb = 1;  // assumed as the channel used for the righteft motor
const int resolution = 8; // 8-bit PWM signal
int dutyCycle =5;
int servoPin = 13;  //the servo is attached to IO_13 on the ESP32
const int servoFrequency = 50;  // 50Hz signal
const int servoChannel = 2;     // channels 0 and 1 are used for the two motors on your EEEBot
const int servoResolution = 12;  // 12-bit PWM signal



void setup() {
  // enable the weak pull up resistors for the two encoders
  //ESP32Encoder::useInternalWeakPullResistors=UP;

  // attach the relevant pins to each encoder
  encoder1.attachHalfQuad(34, 35);
  encoder2.attachHalfQuad(36, 39);

  // set the count of both encoders to 0
  encoder1.setCount(0);
  encoder2.setCount(0);

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution); //servo setup on PWM channel 2, 50Hz, 12-bit (0-4095)

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  Wire.begin(I2C_SLAVE_ADDR);  // Join I2C bus as slave with address I2C_SLAVE_ADDR
  Wire.onReceive(onReceive);   // Register the receive event
  Wire.onRequest(onRequest);    // request event

  Serial.begin(115200);             // Start serial communication
  Serial.println("ESP32 Running");  // Print a message for sanity check
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

void onReceive(int howMany) {
  if (howMany != 6) {
    emptyBuffer();
    return;
  }

  // set up variables for the three 16-bit values
  int16_t leftMotor_speed = 0;
  int16_t rightMotor_speed = 0;
  int16_t servoAngle = 0;
    
  uint8_t servoAngle16_9 = Wire.read();       // receive bits 16 to 9 of z (one byte)
  uint8_t servoAngle8_1 = Wire.read();        // receive bits 8 to 1 of z (one byte)
  uint8_t leftMotor_speed16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t leftMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)
  uint8_t rightMotor_speed16_9 = Wire.read(); // receive bits 16 to 9 of y (one byte)
  uint8_t rightMotor_speed8_1 = Wire.read();  // receive bits 8 to 1 of y (one byte)

  leftMotor_speed = (leftMotor_speed16_9 << 8) | leftMotor_speed8_1;    // combine the two bytes into a 16 bit number
  rightMotor_speed = (rightMotor_speed16_9 << 8) | rightMotor_speed8_1; // combine the two bytes into a 16 bit number
  servoAngle = (servoAngle16_9 << 8) | servoAngle8_1;                   // combine the two bytes into a 16 bit number
  Serial.println(servoAngle);

  Serial.print("Servo Angle: ");
  Serial.println(servoAngle);
  Serial.print("Left Motor Speed: ");
  Serial.println(leftMotor_speed);
  Serial.print("Right Motor Speed: ");
  Serial.println(rightMotor_speed);

  setServoAngle(servoAngle); // Set the servo angle
  setMotorSpeeds(leftMotor_speed, rightMotor_speed); // Set motor speeds

}


void emptyBuffer(void) {
  Serial.println("Error: I2C Byte Size Mismatch");
  while (Wire.available()) {
    Wire.read();
  }
}

void setServoAngle(int servoAngle) {
  // Map servo angle (0-180 degrees) to PWM range (500-2500)
  dutyCycle = map((constrain(servoAngle, 0, 180)), 0, 180, 205, 410); // contrain() limits the minimum and maximum values to 0 and 180 respectively, map() proportionally scales values between 0 and 180 to values between 205 (5% duty cycle) and 410 (10% duty cycle)
  ledcWrite(servoChannel, dutyCycle); // write the control signal to the PWM
}

void setMotorSpeeds(int leftMotor_speed, int rightMotor_speed) {
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

void loop() {
  // continuously 'get' the value from each encoder
  enc1Count = encoder1.getCount();
  enc2Count = encoder2.getCount();
  if (enc1Count >= 26 && enc2Count >= 26){
    setMotorSpeeds(0, 0);
    // set the count of both encoders to 0
    encoder1.setCount(0);
    encoder2.setCount(0);
  }
  
  // can be uncommented for checking what values are returned
  //Serial.println(enc1Count);
  //Serial.print("\t"); // print a 'tab' space between values
  //Serial.println(enc2Count);
}
