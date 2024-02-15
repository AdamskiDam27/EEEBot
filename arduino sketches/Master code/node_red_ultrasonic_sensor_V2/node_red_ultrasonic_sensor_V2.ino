//*****************************************//
//* UON EEEBot 2023                       *//
//* Node Red Communication                *//
//* Ultrasonic Sensor                     *//
// Author - Adam Fairweather              *//
//*****************************************//

//libaries needed for operation
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HCSR04.h>

//address for I2C communication
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

//defining pins for ultrasonic sensor
#define TRIG_PIN 23
#define ECHO_PIN 13
#define LED_PIN 12

//variables for ultrasonic sensor
float duration_us, distance_cm;
int duty;
HCSR04 hc(TRIG_PIN, ECHO_PIN);

//SSID and Password for connecting to the raspberry pi
const char* ssid = "C0117";                    
const char* password = "adam12345";                 
// add your MQTT Broker IP address
//IP ADDRESS NEEDS TO BE CHANGED EACH TIME THE RASPBERRY PI IS REBOOTED                    
const char* mqtt_server = "192.168.137.146";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //configure pins for sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  //setup communciation for I2C
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onRequest);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi(){
  delay(10);
  // we start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  //receives data and changes LED accordingly
  if (String(topic) == "esp32/led") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(LED_PIN, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(LED_PIN, LOW);
    }

  if (String(topic) == "esp32/steering"){
    Serial.print("Changing steering angle");
    if (messageTemp = "left"){
      transmit_steering('l');
    }
    else if (messageTemp == "right"){
      //send steering angle to slave
      transmit_steering('r');
    }
  }
  }
}

void transmit_steering(char direction){
  //transmit the change in direction  
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(direction);
  Wire.endTransmission();
}

void reconnect() {
  // loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // subscribe
      client.subscribe("esp32/led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void onRequest(){
  Wire.write()
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  Wire.beginTransmission

  long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;

    //calculate distance
    distance_cm = hc.dist();
    
    //convert distance_cm to string
    char distance_str[8];
    dtostrf(distance_cm, 1, 2, distance_str);
    
    //send distance to Mqtt
    client.publish("esp32/distance_cm", distance_str);

    delay(5);
  }
}
