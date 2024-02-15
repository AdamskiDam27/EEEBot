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

#define I2C_SLAVE_ADDR 0x04

//defining pins for ultrasonic sensor
#define TRIG_PIN 23
#define ECHO_PIN 13
#define LED_PIN 12

HCSR04 hc(TRIG_PIN, ECHO_PIN);

//SSID and Password for connecting to the raspberry pi
const char* ssid = "C0117";                    
const char* password = "adam12345";                 
// add your MQTT Broker IP address
//IP ADDRESS NEEDS TO BE CHANGED EACH TIME THE RASPBERRY PI IS REBOOTED                    
const char* mqtt_server = "192.168.137.145";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//three integer values are sent to the slave device
int x = 200;
int y = 200;
int z = 90;

int distance_cm;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  //configure pins for sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

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
  }
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

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;

    // two 16-bit integer values are requested from the slave
    int16_t c = 0;
    int16_t d = 0;
    uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
    uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
    uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
    uint8_t b16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
    uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

    c = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
    d = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number

    Serial.print(c);
    Serial.print("\t");
    Serial.println(d);
  
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
    Wire.write((byte)((x & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
    Wire.write((byte)(x & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
    //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
    //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
    Wire.write((byte)((y & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(y & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((z & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(z & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
    Wire.endTransmission();   // stop transmitting
    delay(100);

    //calculate distance
    distance_cm = hc.dist();
    
    //convert encoders to strings
    char encoder1_str[8];
    char encoder2_str[8];

    dtostrf(c, 1, 2, encoder1_str);
    dtostrf(d, 1, 2, encoder2_str);

    Serial.print(encoder1_str);
    Serial.print("\t");
    Serial.println(encoder2_str);

    //convert distance_cm to string
    char distance_str[8];
    dtostrf(distance_cm, 1, 2, distance_str);
    
    //send distance to Mqtt
    client.publish("esp32/distance_cm", distance_str);
    client.publish("esp32/c", encoder1_str);
    client.publish("esp32/d", encoder2_str);

    delay(5);
  }
}
