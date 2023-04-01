#include <Arduino.h>
#include "Logger.h"
#include <Adafruit_GPS.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ecuCAN;

Logger logger;

#define GPSSerial Serial2

Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

void canSniff(const CAN_message_t &msg);
void ecuSniff(const CAN_message_t &msg);

void setup()
{
  logger.setup();
  logger.initializeFile(
    "data",
    {
      "x acceleration (mG)",
      "y acceleration (mG)",
      "z acceleration (mG)",
      "x gyro (mdps)",
      "y gyro (mdps)",
      "z gyro (mdps)",
      "ground speed (knots)",
      "front brake pressure (adcval)",
      "rear brake pressure (adcval)",
      "FR brake rotor temperature (C)",
      "button",
      "RPM",
      "gear",
      "throttle (%)",
      // the last two in this list should always be latitude and longitude
      "latitude",
      "longitude"
    }
  );

  Serial.begin(115200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_BAUD_9600);

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

  ecuCAN.begin();
  ecuCAN.setBaudRate(1000000);
  ecuCAN.setMaxMB(16);
  ecuCAN.enableFIFO();
  ecuCAN.enableFIFOInterrupt();
  ecuCAN.onReceive(ecuSniff);
  ecuCAN.mailboxStatus();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(33, INPUT);
  delay(1000);
}

void canSniff(const CAN_message_t &msg) {
  int frontBrakePressureRaw = 0;
  int rearBrakePressureRaw = 0;
  int xAccel = 0;
  int yAccel = 0;
  int zAccel = 0;
  int xGyro = 0;
  int yGyro = 0;
  int zGyro = 0;
  int frBrakeTempTmp = 0;
  float frBrakeTemperature = 0.0;
  Serial.print("id: ");
  Serial.println(msg.id, HEX);

  switch(msg.id) {
    case 0x360:
    xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "x acceleration (mG)", xAccel);
    logger.addData("data", "y acceleration (mG)", yAccel);
    break;
    case 0x361:
    zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    logger.addData("data", "z acceleration (mG)", zAccel);
    xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "x gyro (mdps)", xGyro);
    break;
    case 0x362:
    yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "y gyro (mdps)", yGyro);
    logger.addData("data", "z gyro (mdps)", zGyro);
    break;
    case 0x363:
    frontBrakePressureRaw = msg.buf[3];       // a8
    frontBrakePressureRaw |= msg.buf[2] << 8;
    rearBrakePressureRaw = msg.buf[5];        // a9
    rearBrakePressureRaw |= msg.buf[4] << 8; 
    logger.addData("data", "front brake pressure (adcval)", frontBrakePressureRaw);
    logger.addData("data", "rear brake pressure (adcval)", rearBrakePressureRaw);
    break;
    
    case 0x364:
    frBrakeTempTmp = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    frBrakeTemperature = (float) ( (float) frBrakeTempTmp / 100.0);
    logger.addData("data", "FR brake rotor temperature (C)", frBrakeTemperature);
    break;
  }
}

void ecuSniff(const CAN_message_t &msg) {
  short rpm = 0;
  short throttleRaw = 0;
  float throttlePct = 0.0;
  short gear = 0;
  switch (msg.id) {
    case 0x360: // RPM, throttle position
    rpm = msg.buf[1];
    rpm |= msg.buf[0] << 8;
    logger.addData("data", "RPM", (float) rpm);

    throttleRaw = msg.buf[5];
    throttleRaw |= msg.buf[4] << 8;
    throttlePct = (float) throttleRaw / 10.0;
    logger.addData("data", "throttle (%)", throttlePct);
    break;
    case 0x470: // gear
    gear = msg.buf[7];
    logger.addData("data", "gear", (float) gear);
    break;
  }
}
char t = 0;
void loop() // run over and over again
{


  Can0.events();
  ecuCAN.events();
  
  char c = GPS.read();
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
      logger.addData("data", "latitude", GPS.latitude);
      logger.addData("data", "longitude", GPS.longitude);
      logger.addData("data", "ground speed (knots)", GPS.speed);
    }
  }

  if(digitalRead(33)) {
    logger.addData("data", "button", (float) 1);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else {
    logger.addData("data", "button", (float) 0);
      // ok status led
    digitalWrite(LED_BUILTIN, HIGH);
  }
  //Serial.println(t);
  if(Serial.available() > 0){
    t = (char) Serial.read();
    //logger.readFile("data");
    digitalWrite(13, LOW);
    File root = SD.open("/");
    logger.printAllFiles(root);
    Serial.println("Done reading");
  }

  static uint32_t timeout = millis();
  if ( millis() - timeout > 5000 ) {
    //Can0.mailboxStatus();
    timeout = millis();

    CAN_message_t msg;
    msg.buf[0] = 0xFF;
    Can0.write(msg);
  }
  logger.writeRow("data");
}