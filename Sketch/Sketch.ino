#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Quaternion quat;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
const unsigned long interval = 100;

void setup(void)
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

	Serial.println("\nUSB connection started");

	/* BNO055 sensor initialization */
	Serial.println("Orientation Sensor Test"); 
  Serial.println("");
	if (!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1);
	}
	delay(1000);
	bno.setExtCrystalUse(true);
  millis();

}

void loop(void)
{
  Serial.flush();
  quat = bno.getQuat();
  currentMillis = millis();

  //Display the orientation data at intervals
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Time (ms): ");
	  Serial.print(currentMillis, DEC);
	  Serial.print(", qw: ");
    Serial.print(quat.w(), 6);
	  Serial.print(", qx: ");
    Serial.print(quat.x(), 6);
    Serial.print(", qy: ");
    Serial.print(quat.y(), 6);
    Serial.print(", qz: ");
    Serial.print(quat.z(), 6);
	  Serial.print(";\n"); 
  }
}

