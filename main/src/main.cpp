#include <Arduino.h>
#include "Logger.h"

Logger logger;
void setup()
{
  logger.setup();
  logger.initializeFile(
    "data",
    {
      "pulse",
    }
  );

  Serial.begin(115200);
  Serial.setTimeout(5000);

  pinMode(5, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(33, INPUT);
  delay(1000);
}

bool written = false;
void loop() // run over and over again
{

  digitalWrite(13, HIGH);

  while(digitalRead(5)){
    if(!written){
      written = true;
      logger.addData("data", "pulse", 1.0);
      logger.writeRow("data");
      Serial.println("p");
    }
    delay(1);
  }

  written = false;
}