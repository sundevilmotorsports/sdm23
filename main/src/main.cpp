#include <Arduino.h>
#include "Logger.h"
#include <Adafruit_GPS.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

Logger logger;

#define GPSSerial Serial2

Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

void canSniff(const CAN_message_t &msg);

void setup()
{
  logger.setup();
  logger.initializeFile(
    "data",
    {
      "x acceleration (mG)",
      "y acceleration (mG)",
      "z acceleration (mG)",
      "ground speed (knots)",
      // the last two in this list should always be latitude and longitude
      "latitude",
      "longitude"
    }
  );

  Serial.begin(115200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_SET_BAUD_9600);

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
}

void canSniff(const CAN_message_t &msg) {
  int reading = 0;
  reading |= msg.buf[0] << 8;
  reading |= msg.buf[1];
  Serial.print("Overrun: " + String(msg.flags.overrun) + "\t");
  Serial.println("CAN data: " + String(reading));
}

void loop() // run over and over again
{
  // ok status led
  digitalWrite(LED_BUILTIN, HIGH);

  Can0.events();
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //Serial.println("new NMEA");
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  //if(millis() - timer > 200){
  if(true){
    //Serial.print("Fix: "); Serial.println((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      //Serial.println("fix!");
      //Serial.print("GPS: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      logger.addData("data", "latitude", GPS.latitude);

      //Serial.print(", ");
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      logger.addData("data", "longitude", GPS.longitude);
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);

      logger.addData("data", "ground speed (knots)", GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
    }
  }

  logger.writeRow("data");
}