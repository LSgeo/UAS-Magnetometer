/*
 * Written by Luke Smith, MQU 2016, part of an MRes project with Tasman and Omar.
 * Ground or airborne magnetometer array with GPS logging. Mega 2560 based board and ebay spec GPS shield
 * Utilisies NeoGPS library, probs shouls mention GNU licence and will do in the final version.
 * 
 * We use NeoHWSerial as it sounds cool and is apparently quicker as it can "handle received characters with a user-defined function during the RX interrupt"
 */

//NeoGPS Variables and Libraries
#include <Arduino.h>
#include "NMEAGPS.h"
#include <NeoHWSerial.h>
#include "rgbStatus.h"
rgbStatus rgb(45,44,46); //initialise rgb led pwm pins on Mega
//#include "Streamers.h" //CSV formatting only for e.g. apparently
#include <SPI.h>
#include <SdFat.h> //Require edit to default SdFatconfig.h for Mega SPI pins and mode
SdFat SD;

File logfile;

NeoHWSerial & gps_port = NeoSerial1; //GPS TX RX port
#define USING_GPS_PORT "NeoSerial1"
#define DEBUG_PORT_TYPE NeoHWSerial //for PC serial monitor.
#define DEBUG_PORT NeoSerial //for PC serial monitor - Serial0
static NMEAGPS gps; //call NMEAGPS via "gps.___"

//LED Status light colours
#define BOOT 255,255,255 //Boot colour - also displays if GPS antenna detached
#define WAITINGFORFIX 100,0,100 //Waiting for fix - displays immediately after set up
#define ACQUIREDFIX 0,200,10 // Why am I even explaining these?
#define DATAFLUSHED 27,109,191 //Courtesy of J.C
#define NOSD 255,0,0
#define FILEERROR 200,100,0
#define DATAOVERRUN 200,0,30

//A number of settings are contained in the accompanying files, such as NMEA sentence chocies, fix and proccessing parameters.

//Pin assignments for LED, magnetometers. LED now in rgbstatus.h
//static const int rPin = 44; //static are visibile to only one function, preserve data between function calls
//static const int gPin = 45; //const makes variable read only
//static const int bPin = 46; //int is an integer

//Chip Select is defined in SdFat

/*Mega 2560 SPI pins:
______|_CS_|_MOSI_|_MISO_|_SCLK_|
Mega  | 53 |  51  |  50  |  52  |
---------------------------------
Shield| 10 |  11  |  12  |  13  |
---------------------------------
*/

//Other set up
#define SENSOR1PIN A8;
#define SENSOR2PIN A9;



//----------------------------------------------------------------
//  Utility to print a long integer like it's a float with 9 significant digits.
//  From NeoGPS library.

void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    outs.print( '-' );
  }

  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );

  // Get fractional degrees
  degE7 -= deg*10000000L;

  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  if (degE7 < 10L)
    outs.print( F("000000") );
  else if (degE7 < 100L)
    outs.print( F("00000") );
  else if (degE7 < 1000L)
    outs.print( F("0000") );
  else if (degE7 < 10000L)
    outs.print( F("000") );
  else if (degE7 < 100000L)
    outs.print( F("00") );
  else if (degE7 < 1000000L)
    outs.print( F("0") );
  
  // Print fractional degrees
  outs.print( degE7 );
}
//---------------------------------------------------------

static void GPSloop() //Function that writes fix info. This constantly runs while we have GPS fix. if gps."not available" 
{
  while (gps.available()) {
  //LED STATUS
  rgb.setColour(WAITINGFORFIX);
    gps_fix fix = gps.read();

    // Log the fix information if we have a location and time

    int sensorValue1 = analogRead(SENSOR1PIN);
    float Voltage1=sensorValue1 * (5.0/1024.0);
    float uT1 = (Voltage1)*50; 

    int sensorValue2 = analogRead(SENSOR2PIN);
    float Voltage2 = sensorValue2 * (5.0/1024.0);
    float uT2 = (Voltage2)*50
    
    if (fix.valid.location && fix.valid.time) {
      rgb.setColour(ACQUIREDFIX); //LED STATUS yay
      static uint16_t lastLoggingTime  = 0;
      uint16_t startLoggingTime = millis();

      printL( logfile, fix.latitudeL() ); //lat
      logfile.print( ',' );
      printL( logfile, fix.longitudeL() ); //lon
      logfile.print(',');
      logfile.print(fix.altitude_cm()); //altitude above ellipsoid, not Mean Sea Level) in int cm
      logfile.print(',');
      
      logfile.print(fix.dateTime.year); //Time. Time is funky for date/time, see neogps documentation.
      logfile.print(fix.dateTime.month); //nb no delimiter between them. 
      logfile.print(fix.dateTime.date);
      logfile.print(',');
      if (fix.dateTime.hours < 10) 
        logfile.print( '0' );
      logfile.print(fix.dateTime.hours);
      logfile.print( ':' );
      if (fix.dateTime.minutes < 10)
        logfile.print( '0' );
      logfile.print(fix.dateTime.minutes);
      logfile.print( ':' );
      if (fix.dateTime.seconds < 10)
        logfile.print( '0' );
      logfile.print(fix.dateTime.seconds);
      logfile.print( '.' );
      if (fix.dateTime_cs < 10)
         logfile.print( '0' ); // leading zero for .05, for example
      logfile.print(fix.dateTime_cs);
      logfile.print(',');

      logfile.print(fix.satellites); //sat count
      logfile.print(',');
      logfile.print(fix.hdop); //hdop  in "integer thousandths of the DOP"
      logfile.print(',');
      logfile.print(fix.lat_err_cm); //lat err i integer centimeters
      logfile.print(',');
      logfile.print(fix.lon_err_cm); //lon err
      logfile.print(',');
      logfile.print(fix.alt_err_cm); //alt err
      logfile.print(',');

      logfile.print(uT1);
      logfile.print(',');


      logfile.print( lastLoggingTime ); // write how long the previous logging took
      logfile.println();


      // flush() is used to empty the contents of the SD buffer to the SD. 
      // If you don't call flush, the data will fill up the SdFat buffer 
      // of 512bytes and flush itself automatically.
      //
      // To avoid losing data or corrupting the SD card file system, you must 
      // call flush() at least once (or close()) before powering down or pulling 
      // the SD card.
      //
      // It is *strongly* recommended that you use some external event 
      // to close the file.  For example, staying within 50m of the moving
      // average location for 1 minute, or using a switch to start and stop 
      // logging.  It would also be good to provide a visual indication 
      // that it is safe to power down and/or remove the card,  perhaps via
      // the LED.
      //
      // To reduce the amount of data that may be lost by an abnormal shut down,
      // you can call flush() periodically.
      //
      // Depending on the amount of data you are printing, you can save 
      // *a lot* of CPU time by not flushing too frequently.  BTW, flushing
      // every time at 5Hz is too frequent.

      // This shows how to flush once a second.
      static uint16_t lastFlushTime = 0;

      if (startLoggingTime - lastFlushTime > 1000) {
        lastFlushTime = startLoggingTime; // close enough
        logfile.flush();
        rgb.setColour(DATAFLUSHED);//display when data flushed
      }

      
      // All logging is finished, figure out how long that took.
      //   This will be written in the *next* record.    
      lastLoggingTime = (uint16_t) millis() - startLoggingTime;
    }
  }
}

//----------------------------------------------------------
void GPSisr( uint8_t c){
  gps.handle( c );
}
//----------------------------------------------------------

static void waitForFix() //Defines what to do while acquiring fix, occurs after setup
{
  DEBUG_PORT.print( F("Waiting for GPS fix...") );
  rgb.setColour(WAITINGFORFIX); 
  uint16_t lastToggle = millis();
  uint16_t currentTime = millis();

  for (;;) { //for ever
    if (gps.available()) {
      if (gps.read().valid.location)
        break; // Got it!
    }

    // Slowly flash the LED until we get a fix
    if (currentTime - lastToggle > 200) {
      rgb.setColour(2,0,20); //Second visible light on boot - if lose fix get this
      DEBUG_PORT.write( '.' );
    }
  }
  DEBUG_PORT.println();
  gps.overrun( false ); // we had to wait a while...
} // waitForFix

//==========================================================
void setup() {
  // Start the normal trace output
    rgb.setColour(BOOT); //First visible light on boot
  DEBUG_PORT.begin(9600); //To Computer - consider removing this for Final RC
  while (!DEBUG_PORT)
    ; // wait for serial port to connect. 
  DEBUG_PORT.println( F("Logging started!") );
  DEBUG_PORT.print( F("fix size = ") );
  DEBUG_PORT.println( sizeof(gps_fix) );
  DEBUG_PORT.print( NMEAGPS_FIX_MAX );
  DEBUG_PORT.println( F(" GPS updates can be buffered.") );
  
  //if (gps.merging != NMEAGPS::EXPLICIT_MERGING)
   //DEBUG_PORT.println( F("Warning: EXPLICIT_MERGING should be enabled for best results!") );

  // Start the GPS device
  gps_port.attachInterrupt( GPSisr ); //GPS on Serial "gps_port"
  gps_port.begin( 9600 );

  //  Configure the GPS.  These are commands for MTK GPS devices. Other brands will have different commands. //No idea why this is here
  //gps.send_P( &gps_port, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ); // RMC only for MTK GPS devices
  //gps.send_P( &gps_port, F("PMTK220,100") ); // 10Hz update rate for MTK GPS devices

DEBUG_PORT.println( F("Initializing SD card...") ); //print to serial monitor
pinMode (53, OUTPUT); //here is where we try to contact the SD, 53 is Source Select on Mega
digitalWrite(53, HIGH); //activate source SPI
  // see if the card is present and can be initialized:
  if (!SD.begin(53)) { //
    
    DEBUG_PORT.println( F("  SD card err - Check SdFat library!") );
    // don't do anything more but Flicker the LED - delay() is ok here because it only occurs on failure
    while (true) {
      rgb.setColour(NOSD);
      delay(200);
      rgb.setColour(10,0,0);
      delay(75); 
      }
    }

  DEBUG_PORT.println( F("  SD card initialized.") );

  // Pick a numbered filename, 00 to 99.
  char filename[15] = "data_##.txt";

  for (uint8_t i=0; i<100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    if (!SD.exists(filename)) {
      // Use this one!
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    DEBUG_PORT.print( F("Couldn't create ") ); 
    DEBUG_PORT.println(filename);
    rgb.setColour(FILEERROR); //Cant create file error

    while (true) {}
  }

  DEBUG_PORT.print( F("Writing to ") ); 
  DEBUG_PORT.println(filename);
  logfile.println( F("lat, lon, alt, YYMMDD, time, sats, HDOP, lat err, lon err, alt err, mT1, mT2, logtime,") );   //time UTC
  
  waitForFix(); //End of Setup, go to waitForFix routine
}

//=================================================================

void loop() { //Constantly run the above GPS Loop
  GPSloop();

  if (gps.overrun()) {
    gps.overrun( false );
    DEBUG_PORT.println( F("DATA OVERRUN: fix data lost!") );
    rgb.setColour(DATAOVERRUN);
  }
}
