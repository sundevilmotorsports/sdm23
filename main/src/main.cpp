#include <Arduino.h>
#include "Logger.h"
#include <Adafruit_GPS.h>
#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 

Logger logger;

// what's the name of the hardware serial port?
#define GPSSerial Serial2

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = 0;


void setup()
{
  logger.setup();
  logger.initializeFile(
    "data",
    {
      "x acceleration (mG)",
      "y acceleration (mG)",
      "z acceleration (mG)",
      "latitude",
      "longitude",
      "ground speed (knots)"
    }
  );
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);


  Wire.begin();

	if( !myISM.begin() ){
		Serial.println("Did not begin.");
		while(1);
	}
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  GPS.sendCommand(PMTK_SET_BAUD_9600);

  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);

  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);
	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 

	Serial.println("Reset.");
	Serial.println("Applying settings.");
	delay(100);
	
	myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
	myISM.setAccelFullScale(ISM_4g); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

}

void loop() // run over and over again
{
  // ok status led
  digitalWrite(LED_BUILTIN, HIGH);

  
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
    timer = millis();
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


  
  	// Check if both gyroscope and accelerometer data is available.
	if( myISM.checkAccelStatus() ){
		myISM.getAccel(&accelData);
    /*
		Serial.print("Accelerometer: ");
		Serial.print("X: ");
		Serial.print(accelData.xData);
		Serial.print(" ");
		Serial.print("Y: ");
		Serial.print(accelData.yData);
		Serial.print(" ");
		Serial.print("Z: ");
		Serial.println(accelData.zData);
    */
    logger.addData("data", "x acceleration (mG)", accelData.xData);
    logger.addData("data", "y acceleration (mG)", accelData.yData);
    logger.addData("data", "z acceleration (mG)", accelData.zData);
	}
  else {
  }

  logger.writeRow("data");
}