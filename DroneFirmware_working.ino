#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#define Pi 3.14159

RF24 radio(9, 10); //CE, CES
byte addresses[][6] = {"0"};

SoftwareSerial gpsSerial(3, 2); //(Rx, Tx)
TinyGPSPlus gps;

struct package {
  float latt = 0.0;
  float lonn = 0.0;
  float magneticX = 0.0;
  float magneticY = 0.0;
  long sonarDistance = 0;
  bool GPS_status = NULL;
  bool compass_status = NULL;
  bool Sonar_status = NULL;
};

typedef struct package Package;
Package data;


unsigned long times = 0;
unsigned long times2 = 0;
int counter = 0;
int counterSonar = 0;
unsigned long start = 0;
unsigned long start2 = 0;

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

float MagMinX = 0, MagMaxX = 0;
float MagMinY = 0, MagMaxY = 0;
float XOffset = 0, YOffset = 0;

void setup() {
  //Start up serial connection
  Serial.begin(115200);

  //Set up GPS Module
  gpsSerial.begin(9600);

  //Set up transceiver
  radio.begin();
  radio.setChannel(108);
  radio.openWritingPipe(addresses[0]);
  radio.setDataRate( RF24_250KBPS);
  radio.setPALevel( RF24_PA_LOW);

  //Initate and check compass connection
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    data.compass_status = false;
  }
  else {
    data.compass_status = true;
  }

  delay(1000);
}

void loop() {

  GetGpsData(); //Get GPS information
  GetDirection(); //Get Compass information
  GetSonar(); //Get sonar sensor information
  radio.write(&data, sizeof(data)); //transmit data through transceiver

}

//Get GPS information
void GetGpsData() {

  bool valid = NULL;
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      valid = gps.location.isValid();
      if (valid && gps.location.age() < 2000) {

        data.latt = gps.location.lat();
        data.lonn = gps.location.lng();
        data.GPS_status = true;
      }
      else {
        data.GPS_status = false;
      }
      counter = 0;
    }
  }
  if (millis() - start >= 1000) {
    counter++;
    start = millis();
  }
  if (counter > 3) {
    data.GPS_status = false;
  }
}


void GetDirection () {

  if (data.compass_status) {
    sensors_event_t event;
    mag.getEvent(&event);

    data.magneticX = event.magnetic.x;
    data.magneticY = event.magnetic.y;
    data.compass_status = true;
  }
  else if (!data.compass_status) {
    if (!mag.begin())
    {
      /* There was a problem detecting the LSM303 ... check your connections */
      data.compass_status = false;
    }
    else {
      data.compass_status = true;
    }
  }
}

void GetSonar () {
  if (Serial.available() > 0) {
    data.sonarDistance = Serial.read();
    if (data.sonarDistance != 0 && data.sonarDistance < 255) {
      data.Sonar_status = true;
      counterSonar = 0;
    }
    else {
      data.Sonar_status = false;
    }
  }
  else if (counterSonar > 3) {
    data.Sonar_status = false;
  }
  if (millis() - start2 > 1000) {
    counterSonar++;
    start2 = millis();
  }
}

