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
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <math.h>

float pitch, roll, yaw;
float deltat;
float lat, lon;
int Year;
byte Month, Day, Hour, Minute, Second;
time_t current_t;

#define PI 3.1415926535897932384626433832795

int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
Serial.print(F("elevat"));
Serial.print("\t");
Serial.print(F("azimut"));
Serial.print("\t");
Serial.print(F("decli"));
Serial.print("\t");
Serial.print(F("HA"));
Serial.print("\t");
Serial.print(F("GMST"));
Serial.print("\t");
Serial.println(F("RA"));
}

void loop() {

// Example usage with provided input values:
lat = 45.665359;   // Latitude of the observer in degrees
lon = 3.100794; // Longitude of the observer in degrees
pitch = 76*DEG_TO_RAD;    // Pitch angle in degrees
roll = 0*DEG_TO_RAD;       // Roll angle in degrees
yaw = 250*DEG_TO_RAD;      // Yaw angle in degrees
Hour = 19; // UTC
Minute = 0;
Second = 14;
Day = 30;
Month = 7;
Year = 2023;

setTime(Hour, Minute, Second, Day, Month, Year);
current_t = now(); 

// Compute the observer's direction in Altitude-Azimuth coordinates
float alt_rad = pitch;
float az_rad = yaw;
//phi : latitude, a : alt, A : az, ST : lst_hours, delta : declination, alpha : right ascension

// Calculate the observer's declination in radians
/*float sindeclination_rad = sin(lat*DEG_TO_RAD)*sin(alt_rad) + cos(lat*DEG_TO_RAD)*cos(alt_rad)*cos(az_rad);
float declination_rad = asin(sindeclination_rad);
float sinhour_angle_rad = -sin(az_rad)*cos(alt_rad) / cos(declination_rad);*/
float declination_rad = altAzTodec(lat*DEG_TO_RAD, alt_rad, az_rad);


// Calculate the Local Sidereal Time (LST) in hours
float lst_hours = calculateLST(lon, current_t);

//lst_hours = 297.92 / 15;
// Calculate the observer's right ascension in radians
float hour_angle_rad = altAzToHA(lat*DEG_TO_RAD, alt_rad, az_rad);

float right_ascension_rad = lst_hours*15*DEG_TO_RAD - hour_angle_rad;

// Convert right ascension and declination from radians to degrees
float right_ascension_deg = right_ascension_rad * RAD_TO_DEG;
float declination_deg = declination_rad * RAD_TO_DEG;

/*if (declination_deg > 90.0) {
  declination_deg = 180.0 - declination_deg;
  right_ascension_deg = fmod(right_ascension_deg + 180.0, 360.0);
} else if (declination_deg < -90.0) {
  declination_deg = -180.0 - declination_deg;
  right_ascension_deg = fmod(right_ascension_deg + 180.0, 360.0);
}*/

// Wrap angles to the range [0, 360) degrees
/*right_ascension_deg = fmod(right_ascension_deg, 360.0);
if (right_ascension_deg < 0) {
  right_ascension_deg += 360.0;
}*/


// Print the calculated celestial coordinates
Serial.print(alt_rad*RAD_TO_DEG);
Serial.print("\t");
Serial.print(az_rad*RAD_TO_DEG);
Serial.print("\t");
Serial.print(declination_deg);
Serial.print("\t");
Serial.print(hour_angle_rad*RAD_TO_DEG);
Serial.print("\t");
Serial.print(lst_hours*15);
Serial.print("\t");
Serial.println(right_ascension_deg);
/*Serial.print("\t");
Serial.print(declination_deg);
Serial.print("\t");
Serial.print(pitch*RAD_TO_DEG,6);
Serial.print("\t");
Serial.print(roll*RAD_TO_DEG,6);
Serial.print("\t");
Serial.print(yaw*RAD_TO_DEG,6);
Serial.print("\t");
Serial.print(lon,6);
Serial.print("\t");
Serial.print(lat,6);
Serial.print("\t");
Serial.print(year(current_t));
Serial.print('-');
Serial.print(month(current_t));
Serial.print('-');
Serial.print(day(current_t));
Serial.print(' ');
Serial.print(hour(current_t));
Serial.print(':');
Serial.print(minute(current_t));
Serial.print(':');
Serial.println(second(current_t));*/
delay(10);
}

// Function to calculate Local Sidereal Time (LST) in hours
// https://aa.usno.navy.mil/faq/GAST#:~:text=The%20Greenwich%20apparent%20sidereal%20time,GAST%20%3D%20GMST%20%2B%20eqeq.
float calculateLST(float longitude_deg, time_t t) {

/*midnight = floor(2458484.833333) + 0.5            // J0 = 2458484.5
days_since_midnight = 2458484.833333 - 2458484.5  // = 0.333333
hours_since_midnight = 0.333333 * 24              // H = 8.0
days_since_epoch = 2458484.833333 - 2451545.0     // D = 6939.833333
centuries_since_epoch = days_since_epoch / 36525  // T = 0.190002
whole_days_since_epoch = 2458484.5 - 2451545.0    // D0 = 6939.5

GMST = 6.697374558 
     + 0.06570982441908 * whole_days_since_epoch 
     + 1.00273790935 * hours_since_midnight
     + 0.000026 * centuries_since_epoch**2        //=470.712605328

GMST_hours = 470 % 24 // = 14
GMST_minutes =  floor(0.712605328 * 60) //=42(.7563197)
GMST_seconds =  0.7563197*60 // =45.38*/
  int H = hour(t);
  double JDTT = computeJulianDate(year(t), month(t), day(t), H);
  Serial.println(JDTT);
  double JD0 = computeJulianDate(year(t), month(t), day(t), 0.0);
  double DTT = JDTT - 2451545.0;
  double DUT = JD0 - 2451545.0;
  double T = DTT / 36525.0;
  double left = 6.697375 + 0.065707485828*DUT + 1.0027379*H + 0.0854103*T + 0.0000258*sq(T);
  double right = 24;
  double GMST = fmod((left + minute(t)/60.0), right);
  float localSiderealTime = GMST + longitude_deg / 15.0;
  return localSiderealTime ;//fmod(localSiderealTime, 360.0) / 15.0; // Convert to hours
}


// Function to compute the Julian Date for a given date
double computeJulianDate(int year, int month, int day, double UT) {
  if (month <= 2) {
    year -= 1;
    month += 12;
  }

  double A = floor(year / 100);
  double B = 2 - A + floor(A / 4);

  double JD = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + B - 1524.5 + UT / 24.0;

  return JD;
}

//Converts Alt/Az to Hour Angle and Declination
//Modified from Meeus so that 0 Az is North
//All angles are in radians
// https://www.astrogreg.com/snippets/altaztoHAdec.html
float altAzToHA(float lat, float alt, float az){
    float H=atan2(-sin(az),tan(alt)*cos(lat)-cos(az)*sin(lat));
    if(H<0){H+=PI*2;}
    return H;
}

float altAzTodec(float lat, float alt, float az){
    float dec=asin(sin(lat)*sin(alt) + cos(lat)*cos(alt)*cos(az));
    return dec;
}

/*
double SiderealPlanets::getGMTsiderealTime(void) {
  modifiedJulianDate1900();
  double days = ((long)(mjd1900 - 0.5)) + 0.5;
  double t = (days / 36525.0) - 1.;
  double r0 = t * (5.13366e-2 + (t * (2.586222e-5 - (t * 1.722e-9))));
  double r1 = 6.697374558 + (2400.0 * (t - ((GMTyear - 2000.0) / 100.0)));
  double t0 = inRange24(r0 + r1);
  GMTsiderealTime = inRange24((GMTtime * 1.002737908) + t0);
  return GMTsiderealTime;
}

GMTtime = GMThour + (GMTminute / 60.0) + (GMTseconds / 3600.0);
GMTyear = year;*/