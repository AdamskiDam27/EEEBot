//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  EEEBot Firmware Code for the Mainboard ESP32        *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

A readme file is commonly posted alongside code in order to explain how to use it.
Always read these before implementing or modifying code in order to understand what
you should/shouldn't change.


DO NOT modify/delete any of the code from the 'EEEBot_MainboardESP32_Firmware' sketch - 
it has been setup with everything you need for Project Week 3. Write your own code for 
the master upstairs ESP32 based off the way the slave mainboard ESP32 is set up. Use 
the example code, both on Moodle and within the Arduino IDE under 'Files --> Examples 
--> Wire' to do this. A skeleton master code has also been provided to act as a template
for code compatible with the firmware code - see 
'Skeleton_Master_Code_Compatible_with_the_Firmware' on Moodle.

You may only change the HIGH/LOW states relating to the motors if the wheels do not
spin in the correct direction i.e. forwards when a positive integer is sent and 
backwards when a negative integer is sent, and the encoder pin order so that the
counts increments and decrements as desired.

1. The mainboard ESP32 slave address is set as 4 (0x04) - use this in your master code.
2. The only data that the mainboard ESP32 slave accepts is six bytes i.e. two bytes per
   signed integer. In your master code, send three signed integer values sequentially.
   See on 'Test Code for I2C Communication Between the ESP32s' and 
   'Skeleton_Master_Code_Compatible_with_the_Firmware' on Moodle for an example of how 
   to do this. 
3. Three integer values are sent to the slave, in the following order: left motor
   speed, right motor speed and steering angle - failure to send these in the correct
   order will affect the EEEBot behaviour.
4. The two integer speed values that are sent to the slave should be between -255 and 
   255. If the value is outside of this range, it will be limited to -255 or 255; 
   whichever is closer. This relates to the PWM value that the motor will run on.
5. The first integer relates to the left motor, the second integer relates to the 
   right motor. Depending on the motor, a PWM value between 0 and ~150 won't be
   sufficient to turn the motor - it is recommended you use PWM values of 200+.
6. A positive value indicates 'forwards' motion on that wheel, a negative value
   indicates a 'backwards' motion on that wheel.
7. The motors will run at a specified speed until instructed otherwise - send 0,0
   in the appropriate format to stop the vehicle.
8. The integer servo angle value should be between 0 and 180. If the value is outside
   of this range, it will be limited to 0 or 180, whichever is closer. This relates to
   the physical angle that the servo can 'turn to'.
9. On request, the mainboard ESP32 slave will send the current encoder count raw value of 
   both encoders to the master - it is NOT set to 0 when this happens, so you need to 
   account for this when working out how many encoder counts have passed.