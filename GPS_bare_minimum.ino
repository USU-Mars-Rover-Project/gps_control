#include <Adafruit_GPS.h>
#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);

// LOCATE TARGET COORDINATE:
double targetLat = 41.741915714505566;
double targetLon = -111.80837907922745;

double GPS_latitude = 0;
double GPS_longitude = 0;

#define GPSECHO false

void setup() {
  Serial.begin(115200);
  
  // INITIALIZE GPS:
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  GPSSerial.println(PMTK_Q_RELEASE); // Ask for firmware version
  
}

void loop() {
  // READ THE GPS
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

    //Serial.print("___>>##>> GPS.latitude: ");
    //Serial.println(GPS.latitude, 12);
    //Serial.print("___>>##>> GPS.longitude: ");
    //Serial.println(GPS.longitude, 12);
    
    int index =  String(GPS.latitude).indexOf('.');
    String start = String(GPS.latitude,12);
    start.remove(index-2);//remove everything after index -2 (the 00.000)
    String ending = String(GPS.latitude,12);
    ending.replace(start,"");//remove the front for the sting 
    //Serial.print("GPS_latitude: ");
    //Serial.println(GPS_latitude,12);

    int index_LON =  String(GPS.longitude).indexOf('.');
    //Serial.print("index_LON: ");
    //Serial.println(index_LON);
    String start_LON = String(GPS.longitude, 12);
    start_LON.remove(index_LON-2);//remove everything after index -2 (the 00.000)
    //Serial.print("start_LON: ");
    //Serial.println(start_LON);
    String ending_LON = String(GPS.longitude, 12);
    ending_LON.replace(start_LON,"");//remove the front for the sting 
    //Serial.print("ending_LON: ");
    //Serial.println(ending_LON);
    
    GPS_latitude = start.toFloat()+ ending.toFloat()/60.0 ;
    GPS_longitude = start_LON.toFloat()+ ending_LON.toFloat()/60.0 ;

    if(GPS.lat == 'S' && GPS_latitude>=0) GPS_latitude = -GPS_latitude;
    if(GPS.lon == 'W' && GPS_longitude>=0) GPS_longitude = -GPS_longitude;

    Serial.print("GPS_latitude: ");
    Serial.println(GPS_latitude,12);
    Serial.print("GPS_longitude: ");
    Serial.println(GPS_longitude,12);
    
    Serial.print("GPS.satellites: ");
    Serial.println(GPS.satellites);
    Serial.println();
  }

}
