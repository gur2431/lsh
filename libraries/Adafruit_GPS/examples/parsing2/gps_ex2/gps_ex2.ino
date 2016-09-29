#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3,2);
Adafruit_GPS GPS(&mySerial);

String NMEA1;
String NMEA2;
char c;
void setup() {
  Serial.begin(115200);
  GPS.begin(96000);
  GPS.sendCommand("$PGCMD,33,0*6");
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommnad(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(1000);
}

void loop() {
  readGPS();

}

void readGPS() {

  while(!GPS.newNMEAReceived()){
  
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NEMA1=GPS.lastNMEA();

    while(!GPS.newNMEAReceived()) {
      c=GPS.read();
    }
    GPS.parse(GPS.lastNMEA());
    NMEA2=GPS.lastNMEA();

    Serial.println(NMEA1);
    Serial.println(NMEA2);
}


