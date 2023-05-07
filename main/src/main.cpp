#include <Arduino.h>
#include "Logger.h"
#include <Adafruit_GPS.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ecuCAN;

Logger logger;
char t = 'z';

#define GPSSerial Serial7
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

void canSniff(const CAN_message_t &msg);
void ecuSniff(const CAN_message_t &msg);

#define ECU_NUM_TX_MAILBOXES 2
#define ECU_NUM_RX_MAILBOXES 6

const float POT_IN_CONVERSION_SCALE = .00210396;


float getLPotValue(const int potPort) {
  float position;
  float valOfNegative = (analogRead(potPort) - 1010);
  float positiveValue = abs(valOfNegative);
  position = .00210396 * (positiveValue); // convert to inches
  position *= 25.4; // convert inches to mm
  return position;
}

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
      "front brake pressure (adcval)",
      "rear brake pressure (adcval)",
      "steering angle (adcval)",
      "FR brake rotor temperature (C)",
      "FR Strain Gauge (adc)",
      "FL Strain Gauge (adc)",
      "RL Damper Position (mm)",
      "RPM",
      "gear",
      "throttle (%)",
      "ground speed (knots)",
      // the last two in this list should always be latitude and longitude
      "latitude",
      "longitude"
    }
  );

  Serial.begin(115200);
  Serial.setTimeout(5000);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  GPS.sendCommand(PMTK_SET_BAUD_9600);

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);

  ecuCAN.begin();
  ecuCAN.setBaudRate(1000000);
  ecuCAN.setMaxMB(ECU_NUM_TX_MAILBOXES + ECU_NUM_RX_MAILBOXES);
  for (int i = 0; i<ECU_NUM_RX_MAILBOXES; i++){
    ecuCAN.setMB((FLEXCAN_MAILBOX)i,RX,EXT);
  }
  for (int i = ECU_NUM_RX_MAILBOXES; i<(ECU_NUM_TX_MAILBOXES + ECU_NUM_RX_MAILBOXES); i++){
    ecuCAN.setMB((FLEXCAN_MAILBOX)i,TX,EXT);
  }
  ecuCAN.setMBFilter(REJECT_ALL);

  ecuCAN.onReceive(MB0, ecuSniff);
  ecuCAN.onReceive(MB1, ecuSniff);
  ecuCAN.setMBFilter(MB0, 0x360);
  ecuCAN.setMBFilter(MB1, 0x470);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(33, INPUT);
  delay(1000);
}

void canSniff(const CAN_message_t &msg) {
  int high = 0, low = 0, mid = 0;
  float frBrakeTemperature = 0.0;
  //Serial.print("id: ");
  //Serial.println(msg.id, HEX);

  switch(msg.id) {
    case 0x360:
    low = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    high = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "x acceleration (mG)", low);
    logger.addData("data", "y acceleration (mG)", high);
    break;
    case 0x361:
    low = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    logger.addData("data", "z acceleration (mG)", low);
    high = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "x gyro (mdps)", high);
    break;
    case 0x362:
    low = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    high = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "y gyro (mdps)", low);
    logger.addData("data", "z gyro (mdps)", high);
    break;
    case 0x363:
    mid = msg.buf[3];
    mid |= msg.buf[2] << 8;
    high = msg.buf[5];
    high |= msg.buf[4] << 8; 
    low = (msg.buf[0] << 8) | msg.buf[1];
    logger.addData("data", "steering angle (adcval)", low);
    logger.addData("data", "front brake pressure (adcval)", mid);
    logger.addData("data", "rear brake pressure (adcval)", high);
    break;
    
    case 0x364:
    low = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    frBrakeTemperature = (float) ( (float) low / 100.0);
    logger.addData("data", "FR brake rotor temperature (C)", frBrakeTemperature);
    break;

    case 0x365:
    low = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    high = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    logger.addData("data", "FR Strain Gauge (adc)", low);
    logger.addData("data", "FL Strain Gauge (adc)", high);
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

void loop() // run over and over again
{

  digitalWrite(13, HIGH);


  Can0.events();
  ecuCAN.events();
  
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  static bool loggedTime = false;
  if (GPS.fix) {
    logger.addData("data", "latitude", GPS.latitude);
    logger.addData("data", "longitude", GPS.longitude);
    logger.addData("data", "ground speed (knots)", GPS.speed);

    if (!loggedTime) {
      logger.logTimestamp(GPS.hour, GPS.minute, GPS.seconds, GPS.year, GPS.month, GPS.day);
      loggedTime = true;
    }
  }

  float rldamper = getLPotValue(A1);
  //Serial.println(rldamper);
  logger.addData("data", "RL Damper Position (mm)", rldamper);
  

  if(Serial.available() > 0){
    t = (char) Serial.read();
    //Serial.println(t);
    digitalWrite(13, LOW);

    if (t == 'a') {
      // download all files
      File root = SD.open("/");
      logger.printAllFiles(root);
    }
    else if (t == 'l') {
      // list files
      logger.listFiles();
    }
    else if (t == 's') {
      // download specific file
      String input = Serial.readStringUntil('-');
      String filename = "run" + input + "-data.csv";
      logger.printFile(filename);

    }
    else {
      Serial.println("garbage");
    }
      Serial.println("Done reading");
  } // end if Serial.available() > 0

  /*
  static uint32_t timeout = millis();
  if ( millis() - timeout > 5000 ) {
    //Can0.mailboxStatus();
    timeout = millis();

    CAN_message_t msg;
    msg.buf[0] = 0xFF;
    Can0.write(msg);
  }
  */
  logger.writeRow("data");
}