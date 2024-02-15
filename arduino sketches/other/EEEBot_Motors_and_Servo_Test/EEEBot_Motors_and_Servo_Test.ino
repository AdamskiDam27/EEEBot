//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  Alex Ottway & Nat Dacombe                           *//
//*  UoN EEEBot 2023                                     *//
//*  Motor & Servo Basic Test Code                       *//
//*                                                      *//
//*  ASSUMPTION: Channel A is LEFT, Channel B is RIGHT   *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// use this code to correctly assign the four pins to move the car forwards and backwards
// you first need to change the pin numbers for the four motor input 'IN' pins and two enable 'en' pins below and then 
// decide which go HIGH and LOW in each of the movements, stopMotors has been done for you
// ** marks where you need to insert the pin number or state

// feel free to modify this code to test existing or new functions

#define enA 33  //EnableA command line
#define enB 25  //EnableA command line

#define INa 26  //Channel A Direction
#define INb 27 //Channel A Direction
#define INc 14 //Channel B Direction
#define INd 12  //Channel B Direction

byte left_speedSetting = 255;
byte right_speedSetting = 150;   //initial speed = 0
byte speedRampFlag = 1;  //define a direction controller for the loop
byte changeDirection = 0;

// setting PWM properties
const int freq = 2000;
const int servoFrequency = 50;
const int ledChannela = 0;
const int ledChannelb = 1;
const int servoChannel = 2;
const int resolution = 8;
const int servoResolution = 12;


int servoPin = 13;
float steeringAngle;  // variable to store the servo position


void setup() {
  // put your setup code here, to run once:

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  // pinMode(enA, OUTPUT);
  // pinMode(enB, OUTPUT);  //if defining some pins as PWM, DON'T set them as OUTPUT!

  // configure LED PWM functionalitites
  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution); //servo setup on PWM2, 50Hz, 12-bit (0-4096)

  //attach the channel to the GPIO to be controlled
 
  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  //initialise Serial Communication
  Serial.begin(9600);
  Serial.println("ESP32 Running");  //sanity Check
}


void loop() {
  // put your main code here, to run repeatedly:

  //this code runs the motors up from zero to full speed and down to zero in both directions
  //the up and down count is controlled by 'speedRampFlag' which cycles negative and positive
  //the change of direction is handled by 'changeDirection'
  //direction change is triggered after a complete runup/rundown cycle (speedSetting reaches zero twice)
  //steering angle is derived from speed setting

  //speedSetting = speedSetting + (1 * speedRampFlag);        //change speed
  //left_speedSetting = 255
  //right_speedSetting = 100
  steeringAngle = map(abs(left_speedSetting), 0, 255, 204, 408);  //create a variable to hold the steering angle, between 5% and 10% duty = 204 - 408 on a 12 bit PWM

  ledcWrite(servoChannel, abs(steeringAngle));  //send our steering angle to the servo.
  
  Serial.print("Steering Angle: ");
  Serial.println(abs(steeringAngle));
  
  //modulus2 ( %2 ) maths effectively asks if changeDirection is odd or even
  if (changeDirection % 2 == 0) {
    goClockwise();
  }  //go forwards

  if (changeDirection % 2 == 1) {
    goClockwise();
  }  //go backwards

  motors(left_speedSetting, right_speedSetting);  //make a call to the 'motors' function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience

  // Serial.print("Motor speeds: ");
  // Serial.println(speedSetting);
}

void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately

  ledcWrite(ledChannela, leftSpeed);
  ledcWrite(ledChannelb, rightSpeed);

  delay(25);
}

void goForwards() {
  //direction is set to 0
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goBackwards() {
  //direction is set to 1
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

// note that the below movement functions are unused in the test code, however you may wise to integrate them into your test code at a later date
void goClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goAntiClockwise() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}
