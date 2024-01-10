/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-ultrasonic-sensor
 */

#include <HCSR04.h>
#define TRIG_PIN 23 // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 13 // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin
#define LED_PIN 12 //ESP31 pin GPIO 16 connected to the LED to show distance

//variables for ultrasonic sensor
float distance_cm;
int duty;
HCSR04 hc(TRIG_PIN, ECHO_PIN);

void setup() {
  // begin serial port
  Serial.begin(9600);

  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
  //configure LED PWM functionalities
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // calculate the distance
  distance_cm = hc.dist();

  //change LED brightness
  duty = set_brightness(distance_cm, duty);
  analogWrite(LED_PIN, duty);

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(500);
}


int set_brightness(float distance_cm, int duty){
  //function for setting the duty cycle of the LED
  if (distance_cm > 100){
    duty = 0;
  }
  else if (distance_cm < 100 && distance_cm > 10){
    duty = 255 - round(distance_cm*2.85);
  }
  else if (distance_cm < 10){
    duty = 255;
  }
  Serial.println(duty);
  return duty;
}
