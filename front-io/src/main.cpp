#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SparkFunMLX90614.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

IRTherm therm;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.mailboxStatus();

  Wire.begin();
  if (!therm.begin()) {
    Serial.println("did not start.");

    while(1); // TODO: remove when done testing
  }

  therm.setUnit(TEMP_C);
  //therm.setEmissivity(0.55); // steel, perhaps

  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  // put your main code here, to run repeatedly:

  int reading = analogRead(A6);

  int brakePressureFrontRaw = analogRead(A8);
  int brakePressureRearRaw = analogRead(A9);

  Serial.print("A8: ");
  Serial.print(brakePressureFrontRaw);
  Serial.print("\tA9: ");
  Serial.print(brakePressureRearRaw);

  float frBrakeTemp = 0;
  if (therm.read()) {
    frBrakeTemp =  therm.object();
    Serial.print("\tAmbient: ");
    Serial.print(therm.ambient());
    Serial.print("\tTemp: ");
    Serial.println(frBrakeTemp);
  }

  Can0.events();

  CAN_message_t msg;
  msg.id = 0x363;

  // steering angle
  msg.buf[0] = (reading & 0xFF00) >> 8;
  msg.buf[1] = reading & 0xFF; // get LSB

  // front (a8 for now)
  msg.buf[2] = (brakePressureFrontRaw & 0xFF00) >> 8;
  msg.buf[3] = brakePressureFrontRaw & 0xFF;

  // rear (a9 for now)
  msg.buf[4] = (brakePressureRearRaw & 0xFF00) >> 8;
  msg.buf[5] = brakePressureRearRaw & 0xFF;
  Can0.write(msg);

  CAN_message_t msg1;
  msg1.id = 0x364;

  // fr brake temp
  /*
  msg1.buf[0] = (frBrakeTemp & 0xFF000000) >> 24;
  msg1.buf[1] = (frBrakeTemp & 0x00FF0000) >> 16;
  msg1.buf[2] = (frBrakeTemp & 0x0000FF00) >> 8;
  msg1.buf[3] = (frBrakeTemp & 0x000000FF);
  */

  Can0.write(msg1);

  delay(20);
}