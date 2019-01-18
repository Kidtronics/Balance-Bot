#ifndef BNO_H
#define BNO_H

/**
 * A helper file that define and setup BNO055 gyroscopic sensor.
*/

#include <Adafruit_BNO055.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 BNO = Adafruit_BNO055(55);

void setupBNO() {
  /* Initialise the sensor */
  if(!BNO.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  BNO.setExtCrystalUse(true);
}

#endif