//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  Skeleton Master Code for Use with the               *//
//*  EEEBot_MainboardESP32_Firmware Code                 *//
//*                                                      *//
//* Written by Nat Dacombe, Edited by Adam Fairweather   *//
//********************************************************//

// the following code acts as a 'bare bones' template for your own custom master code that works with the firmware code provided
// therefore, the variable names are non-descriptive - you should rename these variables appropriately
// you can either modify this code to be suitable for the project week task, or use the functions as inspiration for your own code

#include <Wire.h>
#include <HCSR04.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define TRIG_PIN 23 // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 13 // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
}

// three integer values are sent to the slave device
int left_speed = 0;
int right_speed = 0;
int steering_angle = 90; //0 to 180, 90 is center

int loopCounter = 0;
float distance = 0;

//variables for ultrasonic sensor
float distance_cm;
HCSR04 hc(TRIG_PIN, ECHO_PIN);

//encoder count of left and right encoder
int16_t right = 0;
int16_t left = 0;

void loop(){
  //main loop of program
  if (loopCounter == 0){
    //setting the inital speed conditions
    left_speed = 255;
    right_speed = 255;
    loopCounter++;
  }
  else if (loopCounter == 1){
    //allowing the EEEBot to move forward for 1 second
    delay(1000);

    //setting the speed and steering angle for 180 degree turn
    left_speed = -255;
    right_speed = -255;
    steering_angle = 0;
    loopCounter++;
  }
  else if (loopCounter == 2){
    // delay to allow for turn to execute
    delay(2000); 

    //setting the inital distance value and the speed for reversing
    distance_cm = hc.dist();
    steering_angle = 90;
    left_speed = -130;
    right_speed = -130;
    loopCounter++;
  }
  else if (loopCounter == 3){
    // do while loop to measure distance if the car is not within 10cm of an obstacle
    do{
      distance_cm = hc.dist();
    }
    while(distance_cm > 10);

    //setting speed and steering angle for 90 degree turn
    left_speed = 255;
    right_speed = 255;
    steering_angle = 180;
    loopCounter++;
  }
  else if (loopCounter == 4){
    //allowing the turn to execute
    delay(1000);

    //setting the inital distance value and the speed for reversing
    distance_cm = hc.dist();
    steering_angle = 90;
    left_speed = -130;
    right_speed = -130;
    loopCounter++;
  }
  else if (loopCounter == 5){
    do{
      distance_cm = hc.dist();
    }
    while(distance_cm > 10);
    loopCounter++;
  }
  else{
    left_speed = 0;
    right_speed = 0;
  }

  //transmit and receive all data//

  // two 16-bit integer values are requested from the slave
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  uint8_t b16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
  uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

  if (loopCounter == 2){
    mid_left = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number 
  }
  right = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
  left = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number

  //Serial.print(left);
  //Serial.print("\t");
  //Serial.println(right);
  
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((left_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(left_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((right_speed & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(right_speed & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((steering_angle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(steering_angle & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting
  //delay(100);
}

/*
    if (distance > -43.98){
      distance = (left/46)*18;
      Serial.println(distance);
    }
    else if (distance < -43.98){
      left_speed = 0;
      right_speed = 0;
      steering_angle = 90;
      loopCounter++;
    }
    */
