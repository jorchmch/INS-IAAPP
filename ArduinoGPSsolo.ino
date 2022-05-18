#include <TinyGPS.h>  // *
#include <SoftwareSerial.h>  // *

SoftwareSerial GPS (2, 3);  // * RX TX

unsigned long fix_age, time, speed, course;  // *

TinyGPS gps;  // *
void gpsdump(TinyGPS &gps);  // *
bool feedgps();  // *
void getGPS();  // *
long lat, lon;  // *
float LAT, LON;  // *


void setup() {
  // put your setup code here, to run once:
  GPS.begin(9600);  // *
  Serial.begin(38400);  // *
}

void loop() {
  // put your main code here, to run repeatedly:
  gps.get_position(&lat, &lon, &fix_age);  // *
  getGPS();  // *
  //Serial.print("Latitude : ");
  Serial.print(LAT/1000000,7);
  Serial.print(",");
  Serial.println(LON/1000000,7);
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
  LAT = lat;
  LON = lon;
  {
    feedgps(); // If we don't feed the gps during this long
    //routine, we may drop characters and get checksum errors
  }
}
