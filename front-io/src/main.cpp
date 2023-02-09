#include <Arduino.h>
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.mailboxStatus();

  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  // put your main code here, to run repeatedly:

  int reading = analogRead(A6);

  Can0.events();

  CAN_message_t msg;
  msg.id = 0x363;
  msg.buf[0] = reading >> 8;
  msg.buf[1] = reading & 0xFF;
  Can0.write(msg);
  Serial.println(reading);

  delay(20);
}