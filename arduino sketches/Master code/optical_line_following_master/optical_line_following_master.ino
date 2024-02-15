//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  Skeleton Master Code for Use with the               *//
//*  EEEBot_MainboardESP32_Firmware Code                 *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// the following code acts as a 'bare bones' template for your own custom master code that works with the firmware code provided
// therefore, the variable names are non-descriptive - you should rename these variables appropriately
// you can either modify this code to be suitable for the project week task, or use the functions as inspiration for your own code

#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

//analog pins connected to sensor array from left to right
#define sensor1 35
#define sensor2 32
#define sensor3 33
#define sensor4 25
#define sensor5 26
#define sensor6 27

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
}

// three integer values are sent to the slave device
int left_speed = 130;
int right_speed = 130;
int steering_angle = 52.5;

//values of each sensor
int value1;
int value2;
int value3;
int value4;
int value5;
int value6;

//PID values
int p = 1.3;
int i = 0;
int d = 0;
float k = 0.5;
float u;
float weightedAverage = 0;
float error = 0;
float lasterror = 0;
int errorCounter = 0;

void loop()
{
  //fetch_values(&value1, &value2, &value3, &value4, &value5, &value6);
  value1 = constrain(map(analogRead(sensor1), 3000, 4095, 0, 4095), 0, 4095);
  value2 = constrain(map(analogRead(sensor2), 3398, 4095, 0, 4095), 0, 4095);
  value3 = constrain(map(analogRead(sensor3), 3220, 4095, 0, 4095), 0, 4095);
  value4 = constrain(map(analogRead(sensor4), 3700, 4095, 0, 4095), 0, 4095);
  value5 = constrain(map(analogRead(sensor5), 3210, 4095, 0, 4095), 0, 4095);
  value6 = constrain(map(analogRead(sensor6), 2220, 4095, 0, 4095), 0, 4095);
  
  //Calculate the weigted average
  weightedAverage = ((value1*-37)+(value2*-24)+(value3*-8)+(value4*7)+(value5*24)+(value6*37))/(value1+value2+value3+value4+value5+value6+0.1);

  //error from the weighted average
  error = 37.5 - weightedAverage;

  //Incrementing the error counter
  errorCounter ++;

  //calculating u
  u = (p*error) + (i*(error*errorCounter))+(d*(error-lasterror));

  //altering the sterring angle and motor speeds
  steering_angle = 35 + u;
  left_speed = left_speed + (k*u);
  right_speed = right_speed - (k*u);

  //saving theprevious error
  lasterror = error;

  // two 16-bit integer values are requested from the slave
  int16_t a = 0;
  int16_t b = 0;
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  uint8_t b16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
  uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

  a = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
  b = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number

  //Serial.print(a);
  //Serial.print("\t");
  //Serial.println(b);
  
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
  delay(100);
}
/*
void fetch_values(int* value1, int* value2, int* value3, int* value4, int* value5, int* value6){
  //function designed to read all the analog pins
  value1 = analogRead(sensor1);
  value2 = analogRead(sensor2);
  value3 = analogRead(sensor3);
  value4 = analogRead(sensor4);
  value5 = analogRead(sensor5);
  value6 = analogRead(sensor6);
  }*/