/*RGB Status Function for UAS mag - Source File
 *Remember to #include this in the main ino
 * Luke Smith, MQU, 2016
 * For use in UAS Magnetometer sensor.
 * Pin assignments are hard coded to Mega 2560, but voodoo code remains to get it working.
*/
int _rPin = 22;
int _gPin = 24;
int _bPin = 26;
#include "rgbStatus.h" //Arduino.h is included in the header

//===============================

  rgbStatus::rgbStatus(int _rPin, int _gPin, int _bPin)
  {
    pinMode(24, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(26, OUTPUT);
   }  
  
  void rgbStatus::setColour(int _red, int _green, int _blue)// function called in test, takes values defined in call
  {
    analogWrite(24, 255 - _red); //PWM pin as defined above, write SetColour value (with math).
    analogWrite(22, 255 - _green); //We have a common Cathode LED, so we use 255 - x to conform to srgb definitions.
    analogWrite(26, 255 - _blue);
  }
