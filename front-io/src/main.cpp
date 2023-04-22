#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SparkFunMLX90614.h>
#include <HX711.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

IRTherm therm;
HX711 frSG;
HX711 flSG;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  frSG.begin(9, 10);
  flSG.begin(11, 12);

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.mailboxStatus();

  Wire.begin();
  if (!therm.begin()) {
    Serial.println("did not start."); 

    //while(1); // TODO: remove when done testing
  }

  therm.setUnit(TEMP_C);
  therm.setEmissivity(0.55); // steel, perhaps

  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  // put your main code here, to run repeatedly:

  int reading = analogRead(A6);

  int brakePressureFrontRaw = analogRead(A8);
  int brakePressureRearRaw = analogRead(A9);
/*
  Serial.print("A8: ");
  Serial.print(brakePressureFrontRaw);
  Serial.print("\tA9: ");
  Serial.println(brakePressureRearRaw);
  */

  float frBrakeTemp = 0;
  int outTemp = 0;
  if (therm.read()) {
    frBrakeTemp =  therm.object();
    Serial.print("\tAmbient: ");
    Serial.print(therm.ambient());
    Serial.print("\tTemp: ");
    Serial.println(frBrakeTemp);

    outTemp = frBrakeTemp * 100;
    Serial.println(outTemp);
  }

  int strainGaugeR;
  if(frSG.is_ready()) {
    int strainGaugeR = static_cast<int>(frSG.read());
    Serial.println(strainGaugeR);
  }
  else {
  }

  int strainGaugeL;
  if(flSG.is_ready()) {
    int strainGaugeL = static_cast<int>(flSG.read());
    Serial.println(strainGaugeL);
  }
  else {
  }
  
  Can0.events();

  CAN_message_t msg;
  msg.id = 0x363;

  // steering anglef
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
  msg1.buf[0] = (outTemp & 0xFF000000) >> 24;
  msg1.buf[1] = (outTemp & 0x00FF0000) >> 16;
  msg1.buf[2] = (outTemp & 0x0000FF00) >> 8;
  msg1.buf[3] = (outTemp & 0x000000FF);
  Can0.write(msg1);

  CAN_message_t msg2;
  msg2.id = 0x365;

  // Strain Gauge Right and Left
  msg2.buf[0] = (strainGaugeR & 0xFF000000) >> 24;
  msg2.buf[1] = (strainGaugeR & 0x00FF0000) >> 16;
  msg2.buf[2] = (strainGaugeR & 0x0000FF00) >> 8;
  msg2.buf[3] = (strainGaugeR & 0x000000FF)

  msg2.buf[4] = (strainGaugeL & 0xFF000000) >> 24;
  msg2.buf[5] = (strainGaugeL & 0x00FF0000) >> 16;
  msg2.buf[6] = (strainGaugeL & 0x0000FF00) >> 8;
  msg2.buf[7] = (strainGaugeL & 0x000000FF);
  Can0.write(msg2);

  delay(20);
}