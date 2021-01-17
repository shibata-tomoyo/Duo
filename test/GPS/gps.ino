#include "TinyGPS++.h"

#define RX_PIN 16
#define TX_PIN 17
 
TinyGPSPlus gps;
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);
 
void setup() {
 // Open serial communications and wait for port to open:
 Serial.begin(115200);
 while (!Serial) {
 ; // wait for serial port to connect. Needed for native USB port only
 }
  Serial2.begin(9600,SERIAL_8N1,RX_PIN,TX_PIN);
  Serial2.println("Set the data rate");
}
 
void loop() { // run over and over
 while (Serial2.available() > 0){
  char c = Serial2.read();
  //Serial.print(c);
  gps.encode(c);
  if (gps.location.isUpdated()){
    Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT="); Serial.println(gps.altitude.meters());
  }
 }
}
