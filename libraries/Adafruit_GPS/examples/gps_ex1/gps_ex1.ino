#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
SoftwareSerial GPS(3,2);
Adafruit_GPS GPS(&mySerial);

void setup() {
  GPS.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    GPS.write(Serial.read());
  }
  if(GPS.available()){
    Serial.write(GPS.read());
  }
}

