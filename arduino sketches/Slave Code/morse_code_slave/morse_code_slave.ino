//**********************************//
//* Morse Code Communication       *//
//* Code                           *//
//*                                *//
//*                                *//
//* Slave Device                   *//
//**********************************//

#include <Wire.h>        // include Wire library

String message;

void setup(){
  Wire.begin(0x01);             // join i2c bus with address 8
  Wire.onReceive(receiveEvent); // create a receive event
  Serial.begin(9600);           // start serial to visualise data 
}

void loop() {
}

void receiveEvent(int howMany){
  message = "";
  while (Wire.available()){  // loop whilst bus is busy
    char c = Wire.read();     // receive data byte by byte
    message += c;             // form complete string
  }
  if (message == ".. .----. -- / .. -. / - .... . / .-- .. .-. . ... --..-- / .... . .-.. .--. -.-.-- -.-.--"){
    Serial.println("The message is .. .----. -- / .. -. / - .... . / .-- .. .-. . ... --..-- / .... . .-.. .--. -.-.-- -.-.--");
    Serial.println("In English, it is: I'm in the wires, help!");
  }
  else if (message == "- .... .. ... / .. ... / -- --- .-. ... . / -.-. --- -.. ."){
    Serial.println("The message is - .... .. ... / .. ... / -- --- .-. ... . / -.-. --- -.. .");
    Serial.println("In English, it is: This is morse code");
  }
  else if (message == ". .-.. . -.-. - .-. --- -. .. -.-. / .- -. -.. / . .-.. . -.-. - .-. .. -.-. .- .-.. / . -. --. .. -. . . .-. .. -. --."){
    Serial.println("The message is . .-.. . -.-. - .-. --- -. .. -.-. / .- -. -.. / . .-.. . -.-. - .-. .. -.-. .- .-.. / . -. --. .. -. . . .-. .. -. --.");
    Serial.println("In English, it is: Electronic and Electrical Engineering");
  }
  else if (message == ".. / .-.. --- ...- . / . -. --. .. -. . . .-. .. -. --."){
    Serial.println("The message is .. / .-.. --- ...- . / . -. --. .. -. . . .-. .. -. --.");
    Serial.println("In English, it is: I love Engineering");
  }
  else if (message == "--- ... -.-. .. .-.. .-.. --- ... -.-. --- .--. . ..."){
    Serial.println("The message is --- ... -.-. .. .-.. .-.. --- ... -.-. --- .--. . ...");
    Serial.println("In English, it is: Oscilloscopes");
  }
  else if (message == "...- --- .-.. - -- . - . .-."){
    Serial.println("The message is ...- --- .-.. - -- . - . .-.");
    Serial.println("In English, it is: Voltmeter");
  }
  //Serial.println(message);    // write string to serial monitor
  delay(500);
}