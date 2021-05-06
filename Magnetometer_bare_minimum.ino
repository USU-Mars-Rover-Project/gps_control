#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
//#include <Wire.h>

// COMPASS (MAGNETOMETER) CALIBRATION: 
//     SET THESE TO ZERO BEFORE YOU CALIBRATE AGAIN!!!
float magX_min = -14.27;  // Obtain this value from calibration sketch
float magY_min = -46.64;  // Obtain this value from calibration sketch
float magZ_min = -1;      // Obtain this value from calibration sketch
float magX_max = 45.45;   // Obtain this value from calibration sketch
float magY_max = 11.91;   // Obtain this value from calibration sketch
float magZ_max = 1;       // Obtain this value from calibration sketch
bool useCalibratedMag = true;  // uses the min and max mag values, else uses magnitudes of 1.
float heading = 0;
bool calibrating = false; // Switch this to true to print max and min mag values.
float magneticDeclination = 30; // use this to correct for any bias errors from true north

Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

void setup() {
  Serial.begin(115200);
  
  // INITIALIZE MAGNETOMETER:
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  delay(1000);

}

void loop() {
  while(calibrating){
    sensors_event_t event;
    mag.getEvent(&event);
    // Find all minimum and maximum mag values:
    if (event.magnetic.x < magX_min) magX_min = event.magnetic.x;
    if (event.magnetic.x > magX_max) magX_max = event.magnetic.x;
    if (event.magnetic.y < magY_min) magY_min = event.magnetic.y;
    if (event.magnetic.y > magY_max) magY_max = event.magnetic.y;
    if (event.magnetic.z < magZ_min) magZ_min = event.magnetic.z;
    if (event.magnetic.z > magZ_max) magZ_max = event.magnetic.z;
    Serial.print("x: ");
    Serial.print(event.magnetic.x);
    Serial.print("\ty: ");
    Serial.println(event.magnetic.y);
    // Print all minimum and maximum mag values (don't need z):
    Serial.print("minXY: ");
    Serial.print(magX_min);
    Serial.print(" ");
    Serial.println(magY_min);
    //Serial.print(" ");
    //Serial.println(magZ_min);
    Serial.print("maxXY: ");
    Serial.print(magX_max);
    Serial.print(" ");
    Serial.println(magY_max);
    Serial.println(" ");
    //Serial.println(magZ_max);
  }
  // READ THE MAGNETOMETER:
  sensors_event_t event;
  mag.getEvent(&event);
  // Calculate the angle of the vector y,x
  if(useCalibratedMag){
    float mag_x = mapf(event.magnetic.x, magX_min,magX_max, -100,100);
    float mag_y = mapf(event.magnetic.y, magY_min,magY_max, -100,100);
    heading = ((atan2(mag_y, mag_x) * 180) / PI) + magneticDeclination;
  }
  else{
    heading = ((atan2(event.magnetic.y, event.magnetic.x) * 180) / PI) + magneticDeclination;
  }
  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }

  Serial.print("heading: ");
  Serial.println(heading);
}


double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
