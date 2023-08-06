/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include "SensorFusion.h" //SF
SF fusion;

float mx, my, mz, mx_off, my_off, mz_off, mx_cal, my_cal, mz_cal;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();

  mx_off = mx - 4.642259;
  my_off = my - 32.417171;
  mz_off = mz - 18.6896;

  mx_cal = 1.127426*mx_off + -0.003916*my_off + -0.061371*mz_off;
  my_cal = -0.003916*mx_off + 1.032829*my_off + -0.035828*mz_off;
  mz_cal = -0.061371*mx_off + -0.035828*my_off + 1.280902*mz_off;

  Serial.print(mx,6);
  Serial.print("\t");
  Serial.print(my,6);
  Serial.print("\t");
  Serial.print(mz,6);
  Serial.print("\t");
  Serial.print(mx_cal,6);
  Serial.print("\t");
  Serial.print(my_cal,6);
  Serial.print("\t");
  Serial.print(mz_cal,6);
  Serial.print("\t");
  
  Serial.println(IMU.getTemperature_C(),6);

  delay(10);
}
