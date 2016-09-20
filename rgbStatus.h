/* RGB Status Function for UAS mag - Header File
 * Remember to #include this in the main ino
 * Luke Smith, MQU, 2016
 * For use in UAS Magnetometer sensor.
 * Pin assignments are hard coded to Mega 2560, but voodoo code remains to get it working.
*/

#ifndef rgbStatus_h
#define rgbStatus_h
#include "Arduino.h"

class rgbStatus 
{
  public:
    rgbStatus(const int _rPin, const int _gPin, const int _bPin); //List all fucntions in RGBStatus.cpp here
    void setColour(int _red, int _green, int _blue); //

  private:
    int _rPin, _gPin, _bPin; //Holds each colour pin to output too, constant
    int _red, _green, _blue; //holds srgb value we wish to output 
    };

#endif
