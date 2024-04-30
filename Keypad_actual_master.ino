#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//Adafruit_MPU6050 mpu;
//Adafruit_Sensor *mpu_gyro;

#define I2C_SLAVE_ADDR 0x04  // I2C slave address

const byte ROWS = 4;
const byte COLS = 3;

char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};

byte rowPins[ROWS] = { 2, 0, 4, 16 };
byte colPins[COLS] = { 17, 5, 18 };

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

const int L = 100;
char KeyArray[L];
int ArrayIndex = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(19, 23, 33, 15, 32, 27);

int rightMotor_speed = 0;
int leftMotor_speed = 0;
int servoAngle = 90;
int encoder1Count = 0;
int encoder2Count = 0;
float turningAngle = 0;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Command entered:");

  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 1);

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

  Serial.print(a);
  Serial.print("\t");
  Serial.println(b);

  lcd.setCursor(0, 1);

  
  char key = customKeypad.getKey();  // Read the pressed key
  Serial.println(key);
  
  if (key != NO_KEY) {
    if(key == '#') {
      lcd.clear();
      lcd.print("Running...");
      Serial.print("Value in Array = ");
      printArray();
      Serial.println("");
      executeArray();
      lcd.clear();
      lcd.print("Command entered:");
    } else if (key == '*') {
      clearArray();
    } else {
      addToArray(key);
      printCommand(key);
    }
  }
}


void executeArray(){
  int i = 0;

  while (KeyArray[i] != '\0') {
    char currentCommand = KeyArray[i];
    switch (currentCommand) {
      case '2':
        rightMotor_speed = leftMotor_speed = 140;
        servoAngle = 90;
        Serial.println(servoAngle);
        Serial.println("Forward");
        break;
      case '8':
        rightMotor_speed = leftMotor_speed = -140;
        servoAngle = 90;
        Serial.println("Reversing");
        break;
      case '4':
        servoAngle = 0;
        rightMotor_speed = 170;
        leftMotor_speed = 170;
        Serial.println("Turn left");
        break;
      case '6':
        servoAngle = 180;
        rightMotor_speed = 170;
        leftMotor_speed = 170;
        Serial.println("Turn right");
        break;
      case '5':
        servoAngle = 90;
        leftMotor_speed = rightMotor_speed = 0;
        Serial.println("Set servo straight");
        break;
      case '*':
        servoAngle = 90;
        leftMotor_speed = rightMotor_speed = 0;
        Serial.println("Command chain complete");
        break;
      default:
        Serial.println("No command assigned to key");
        break;
    }

    //lcd.print(currentCommand);

    // Send commands to the slave
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(servoAngle >> 8);
    Wire.write(servoAngle & 0xFF);
    Wire.write(leftMotor_speed >> 8);
    Wire.write(leftMotor_speed & 0xFF);
    Wire.write(rightMotor_speed >> 8);
    Wire.write(rightMotor_speed & 0xFF);
    Wire.endTransmission();
    
    // Delay for 2 seconds before moving to the next command
    delay(1000);
    i++; // Move to the next command
    
  }
  
  // Clear the array after all functions have been executed
  clearArray();
}


void addToArray(char value) {
  KeyArray[ArrayIndex] = value;
  ArrayIndex = (ArrayIndex + 1) % L; // Corrected to L
}

void clearArray() {
  for (int i = 0; i < L; i++) {
    KeyArray[i] = '\0';
  }
  ArrayIndex = 0;
}

void printArray() {
  for (int i = 0; i < L; i++) {
    int actualIndex = (ArrayIndex - L + i + L) % L;
    Serial.print(KeyArray[actualIndex]);
  }
}

void printCommand(char value){
  lcd.clear();
  lcd.print("Command entered:");
  lcd.setCursor(0, 1);

  switch (value) {
      case '2':
          lcd.print("Forwards");
        break;
      case '8':
        lcd.print("Backwards");
        break;
      case '4':
        lcd.print("Left turn");
        break;
      case '6':
        lcd.print("Right turn");
        break;
      case '5':
        lcd.print("End of commands");
        break;
  }
}