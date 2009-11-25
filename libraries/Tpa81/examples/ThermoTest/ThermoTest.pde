// This is a simple example of how to use the TPA81 library
#include <Wire.h>
#include <Tpa81.h>

// Create an instance of Tpa81, constructor is Tpa81(deviceID)
Tpa81 thermo = Tpa81(0);

void setup(){		// this is called once
  Wire.begin();   // this must be called to start the I2C bus
  Serial.begin(38400);
}

void loop(){        // print a reading every 2 seconds
  int i;
  unsigned char reading[8];

  // query the sensor
  i = thermo.getData(reading);
    
  // send up data to PC like "AMBIENT: p1, p2.... p8.\n"
  Serial.print(i);    // print the ambient temperature;
  Serial.print(": ");
  Serial.print((int)reading[0]);
  for(i=1;i<8;i++){
    Serial.print(",");
    Serial.print((int)reading[i]);
  }
  Serial.println(".");
  delay(2000);	
}
