/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


#define LEFTDRIVE 11
#define RIGHTDRIVE 10
#define GPSSerial Serial1

double targetLat = 41.741915714505566;
double targetLon = -111.80837907922745;
double GPS_latitude = 0;
double GPS_longitude = 0;

float steeringAngle = 0;
float distFromTarget = 0;

float leftThrottle = 0;   // 1000 to 2000, 1500 is stopped, 0 is disarmed
float rightThrottle = 0;  // 1000 to 2000, 1500 is stopped, 0 is disarmed
float rest = 1500;        // resting PWM
float driveSpeed = 200;
float maxCCW = rest+driveSpeed;      // clockwise PWM
float maxCW = rest-driveSpeed;       // counter clockwise PWM

float magX_min = -19.00;  // Obtain this value from calibration sketch
float magY_min = -31.09;  // Obtain this value from calibration sketch
float magZ_min = -1;  // Obtain this value from calibration sketch
float magX_max = 68.82;   // Obtain this value from calibration sketch
float magY_max = 52.18;   // Obtain this value from calibration sketch
float magZ_max = 1;   // Obtain this value from calibration sketch
bool useCalibratedMag = true;
float heading = 0;
bool calibrating = false; // press button 1 to enter calibration mode, button 2 to exit.
unsigned long blePrintMillis = 0;
unsigned long buttonMillis = 0;
bool autonomous = false;

Servo left;
Servo right;

Adafruit_GPS GPS(&GPSSerial);
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

#define GPSECHO false

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  left.attach(LEFTDRIVE);
  right.attach(RIGHTDRIVE);
  left.writeMicroseconds(0);
  right.writeMicroseconds(0);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  GPSSerial.println(PMTK_Q_RELEASE); // Ask for firmware version

  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  
}
 float round_to_dp( float in_value, int decimal_place )
{ //https://forum.arduino.cc/t/float-rounding-and-truncate-functions/320002/9
    float multiplier = powf( 10.0f, decimal_place );
    in_value = roundf( in_value * multiplier ) / multiplier;
    return in_value;
}
/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{

  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another

    Serial.print("___>>##>> GPS.latitude: ");
    Serial.println(GPS.latitude, 12);
    Serial.print("___>>##>> GPS.longitude: ");
    Serial.println(GPS.longitude, 12);
    
    int index =  String(GPS.latitude).indexOf('.');
    String start = String(GPS.latitude,12);
    start.remove(index-2);//remove everything after index -2 (the 00.000)
    String ending = String(GPS.latitude,12);
    ending.replace(start,"");//remove the front for the sting 
    Serial.print("GPS_latitude: ");
    Serial.println(GPS_latitude,12);

    int index_LON =  String(GPS.longitude).indexOf('.');
    Serial.print("index_LON: ");
    Serial.println(index_LON);
    String start_LON = String(GPS.longitude, 12);
    start_LON.remove(index_LON-2);//remove everything after index -2 (the 00.000)
    Serial.print("start_LON: ");
    Serial.println(start_LON);
    String ending_LON = String(GPS.longitude, 12);
    ending_LON.replace(start_LON,"");//remove the front for the sting 
    Serial.print("ending_LON: ");
    Serial.println(ending_LON);
    
    GPS_latitude = start.toFloat()+ ending.toFloat()/60.0 ;
    GPS_longitude = start_LON.toFloat()+ ending_LON.toFloat()/60.0 ;

    if(GPS.lat == 'S' && GPS_latitude>=0) GPS_latitude = -GPS_latitude;
    if(GPS.lon == 'W' && GPS_longitude>=0) GPS_longitude = -GPS_longitude;

    //Serial.print("GPS_latitude: ");
    //Serial.println(GPS_latitude,12);
    Serial.print("GPS_longitude: ");
    Serial.println(GPS_longitude,12);
    
    Serial.print("GPS.satellites: ");
    Serial.println(GPS.satellites);
    Serial.println();
  }

  sensors_event_t event;
  mag.getEvent(&event);
  float pi = 3.14159;
  // Calculate the angle of the vector y,x
  if(useCalibratedMag){
    float mag_x = mapf(event.magnetic.x, magX_min,magX_max, -100,100);
    float mag_y = mapf(event.magnetic.y, magY_min,magY_max, -100,100);
    heading = (atan2(mag_y, mag_x) * 180) / pi;
  }
  else{
    heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / pi;
  }
  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }

  steeringAngle = getSteering(heading, GPS_latitude, GPS_longitude, targetLat, targetLon);
  distFromTarget = getDistance(GPS_latitude, GPS_longitude, targetLat, targetLon); // meters
  
  if(autonomous && GPS.satellites >= 3){
    if(distFromTarget > 10){
      differentialDrive(steeringAngle, driveSpeed);
    }
    else{
      stopDrive();
    }
  }

  
  if(!calibrating){
    if(millis()-blePrintMillis > 1000){
      blePrintMillis = millis();
      ble.print(heading);
      ble.print(" ");
      ble.print(steeringAngle);
      ble.print(" ");
      ble.print(distFromTarget);
      ble.print(" ");
      ble.print(GPS.satellites);
      ble.print(" ");
      ble.print(GPS_latitude,12);
      ble.print(" ");
      ble.println(GPS_longitude,12);
    }
  }
  else{
    // Find all minimum and maximum mag values:
    if (event.magnetic.x < magX_min) magX_min = event.magnetic.x;
    if (event.magnetic.x > magX_max) magX_max = event.magnetic.x;
    if (event.magnetic.y < magY_min) magY_min = event.magnetic.y;
    if (event.magnetic.y > magY_max) magY_max = event.magnetic.y;
    if (event.magnetic.z < magZ_min) magZ_min = event.magnetic.z;
    if (event.magnetic.z > magZ_max) magZ_max = event.magnetic.z;
    // Print all minimum and maximum mag values:
    ble.print("minXY: ");
    ble.print(magX_min);
    ble.print(" ");
    ble.println(magY_min);
    //ble.print(" ");
    //ble.println(magZ_min);
    ble.print("maxXY: ");
    ble.print(magX_max);
    ble.print(" ");
    ble.println(magY_max);
    ble.println(" ");
    //ble.println(magZ_max);
  }
  
  
  if(!autonomous){  // poll the buttons repeatedly
    /* Wait for new data to arrive */
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len == 0) return;
  }
  else if(autonomous && millis()-buttonMillis > 100){  // poll the buttons once per 100ms
    buttonMillis = millis();
    /* Wait for new data to arrive */
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len == 0) return;
  }


  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
      // 5=forward, 6=back, 7=leftTurn, 8=rightTurn
      // leftPWM=10 rightPWM=11
      if(buttnum==5 && !autonomous){ // All forward
        left.writeMicroseconds(maxCCW);
        right.writeMicroseconds(maxCW);
      }
      if(buttnum==6 && !autonomous){ // All backward
        left.writeMicroseconds(maxCW);
        right.writeMicroseconds(maxCCW);
      }
      if(buttnum==7 && !autonomous){ // right forward, left backward
        left.writeMicroseconds(maxCW);
        right.writeMicroseconds(maxCW);
      }
      if(buttnum==8 && !autonomous){ // right backward, left forward
        left.writeMicroseconds(maxCCW);
        right.writeMicroseconds(maxCCW);
      }
      if(buttnum==1){ // enter magnetometer calibration mode
        calibrating = true;
        ble.println("CALIBRATING MAG");
      }
      if(buttnum==2){ // exit magnetometer calibration mode
        calibrating = false;
        ble.println("DONE CALIBRATING");
      }
      if(buttnum==3){ // enter autonomous mode
        autonomous = true;
        ble.println("AUTONOMOUS");
      }
      if(buttnum==4){ // exit autonomous mode
        autonomous = false;
        stopDrive();
        ble.println("MANUAL");
      }
    } else {
      if(buttnum==4){ // exit autonomous mode
        autonomous = false;
        stopDrive();
        ble.println("MANUAL");
      }
      Serial.println(" released");
      // All stop
      left.writeMicroseconds(rest);
      right.writeMicroseconds(rest);
    }
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
//    mag = atan2(y,x)*180/3.1415926; //-magCal;   // Store into the global variable;
    Serial.print("Mag\t");
    //Serial.print(x); Serial.print('\t');
    //Serial.print(y); Serial.print('\t');
    //Serial.print(z); Serial.println();
    //Serial.println(mag);
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
}


double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float deg2rad(float deg){
  return deg*PI/180;
}

float rad2deg(float rad){
  return rad*180/PI;
}

float getDistance(double lat1, double lon1, double lat2, double lon2){  // generally used geo measurement function
    float R = 6378.137; // Radius of earth in KM
    float dLat = lat2 * PI / 180 - lat1 * PI / 180;
    float dLon = lon2 * PI / 180 - lon1 * PI / 180;
    float a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * PI / 180) * cos(lat2 * PI / 180) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float d = R * c;
    return d * 1000; // meters
}

float getSteering(float currHead, float currLat, float currLon, float targLat, float targLon){
    // currHead must be from -360 to 360;
    float x = cos(deg2rad(targLat)) * sin(deg2rad(currLon-targLon));
    float y = cos(deg2rad(currLat)) * sin(deg2rad(targLat)) - sin(deg2rad(currLat)) * cos(deg2rad(targLat)) * cos(deg2rad(currLon-targLon));
    float steering = -rad2deg(atan2(x,y)) - currHead;
    if(steering >= 180){
        steering = steering - 360;
    }
    if(steering <= -180){
        steering = steering + 360;
    }
    return steering;
}


void differentialDrive(float steeringAngle, float driveSpeed){
    // steeringAngle must be from -180 to 180.
    // maximum forward is 2000.
    // stop is 1500.
    // maximum reverse is 1000.
    float leftDriveSpeed = rest+driveSpeed;  // 1700
    float rightDriveSpeed = rest-driveSpeed; // 1300
    if (steeringAngle < 0){
      float L = leftDriveSpeed - mapf(abs(steeringAngle), 0,180, 0,200); // slow left down
      float R = rightDriveSpeed;
      left.writeMicroseconds(L);
      right.writeMicroseconds(R);
    }
    if (steeringAngle > 0){
      float L = leftDriveSpeed;
      float R = rightDriveSpeed + mapf(abs(steeringAngle), 0,180, 0,200); // slow right down
      left.writeMicroseconds(L);
      right.writeMicroseconds(R);
    }
}

void stopDrive(){
  left.writeMicroseconds(rest);
  right.writeMicroseconds(rest);
}
