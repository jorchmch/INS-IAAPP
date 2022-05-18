#include <TinyGPS.h>  // *
#include <SoftwareSerial.h>  // *
#include <Wire.h>

#define FLOATS_SENT 3
#define SLAVE_ADDRESS 0x08
float data[FLOATS_SENT];

SoftwareSerial GPS (2, 3);  // *RX TX 


unsigned long fix_age, time, speed, course;  // *

TinyGPS gps;  // *
void gpsdump(TinyGPS &gps);  // *
bool feedgps();  // *
void getGPS();  // *
long lat, lon;  // *
long alt; //new
float LAT, LON;  // *
float ALT; //new

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  // put your setup code here, to run once:
  GPS.begin(9600);  // *
  Serial.begin(38400);  // *
  Wire.onRequest(requestEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  gps.get_position(&lat, &lon, &fix_age);  // *
  getGPS();  // *
  //Serial.print("Latitude : ");
   
  Serial.print(LAT/1000000,8);
  Serial.print(",");
  Serial.print(LON/1000000,8);
  Serial.print(",");
  Serial.println(ALT);
}

void getGPS(){
  bool newdata = false;
  unsigned long start = millis();
  // Every 1 seconds we print an update
  while (millis() - start < 1000)
  {
    if (feedgps ()){
    newdata = true;
    }
  }
  if (newdata)
  {
    gpsdump(gps);
  }
}

bool feedgps(){
  while (GPS.available())
  {
    if (gps.encode(GPS.read()))
    return true;
  }
  return 0;
}
void gpsdump(TinyGPS &gps)
{
  //byte month, day, hour, minute, second, hundredths;
  gps.get_position(&lat, &lon);
  ALT = gps.f_altitude(); // new
  LAT = lat;
  LON = lon;
  
  {
    feedgps(); // If we don't feed the gps during this long
    //routine, we may drop characters and get checksum errors
  }
}
void requestEvent() {
  data[0] = LAT/1000000; // latitude
  data[1] = LON/1000000; // longitude
  data[2] = ALT;  // altitude
  //data[2] = 1111;
  Wire.write((byte*) &data, FLOATS_SENT*sizeof(float));
  //Wire.write('h');

}
