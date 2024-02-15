//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  ESP32 MQTT EEEBot Template                          *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// the following code is modified from https://randomnerdtutorials.com by Rui Santos

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>


// add your required sensor/component libraries here
// --
// --

//defining pins for the ultrasnboic sensor
#define TRIG_PIN 23
#define ECHO_PIN 13
#define LED_PIN 12

float duration_us, distance_cm;
int duty;

// replace the next variables with your SSID/Password combination
const char* ssid = "C0117";                      //CHANGE ME
const char* password = "adam12345";              //CHANGE ME     
// add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";                      
const char* mqtt_server = "192.168.137.125";          //CHANGE ME

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


void setup() {
  Serial.begin(9600);

  //configure pins for sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
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

  // add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
  // --
}

void reconnect() {
  // loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      
      // add your subscribe topics here
      // --
      client.subscribe("esp32/output");
      // --
         
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    
    // add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --

    //generate a 10-microsecond pulse to TRIG pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    //measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);

    //calculate distance
    distance_cm = 0.017 * duration_us;

    char distance_char[8];
    dtostrf(distance_cm, 1, 2, distance_char);
    client.publish("esp32/distance_cm", distance_char);

    delay(500);
    // --
  }
}
