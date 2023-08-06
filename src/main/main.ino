#include <MPU9250.h>
#include <SensorFusion.h> //SF
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif


SF fusion;
LiquidCrystal_I2C lcd(0x27, 20, 4);
static const byte RXPin = 11, TXPin = 12;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

#define PI 3.1415926535897932384626433832795

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);

int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}
  // start communication with GPS
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    lcd.print(F("Check IMU wiring or try cycling power"));
    /*Serial.println(F("IMU initialization unsuccessful"));
    Serial.println(F("Check IMU wiring or try cycling power"));
    Serial.print(F("Status: "));
    Serial.println(status);*/
    while (1)
    {
    }
  }
  static const uint32_t GPSBaud = 9600;
  ss.begin(GPSBaud);
  /*Serial.print(F("elevat"));
  Serial.print("\t");
  Serial.print(F("azimut"));
  Serial.print("\t");
  Serial.print(F("decli"));
  Serial.print("\t");
  Serial.print(F("HA"));
  Serial.print("\t");
  Serial.print(F("GMST"));
  Serial.print("\t");
  Serial.println(F("RA"));*/

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(F("RA:"));
  lcd.setCursor(9,0);
  lcd.print(F("Dec:"));
  lcd.setCursor(0,2);
  lcd.print(F("Elev:"));
  lcd.setCursor(9,2);
  lcd.print(F("Azim:")); 
  lcd.setCursor(16,0);
  lcd.print(F("Time"));
  
  lcd.setCursor(19,1);
  lcd.print(F("h"));
  lcd.setCursor(19,2);
  lcd.print(F("m"));
  lcd.setCursor(19,3);
  lcd.print(F("s"));

  lcd.setCursor(6,1);
  lcd.printByte(223);
  lcd.setCursor(14,1);
  lcd.printByte(223);
  lcd.setCursor(6,3);
  lcd.printByte(223);
  lcd.setCursor(14,3);
  lcd.printByte(223);

}

void loop() {

  float ax_cal, ay_cal, az_cal, gx_cal, gy_cal, gz_cal;
  float mx_off, my_off, mz_off, mx_cal, my_cal, mz_cal;
  float pitch, roll, yaw;
  float deltat;
  float lat, lon;
  int Year;
  byte Month, Day, Hour, Minute, Second;
  // read the sensor
  IMU.readSensor();
  // display the data
  ax_cal =  0.9976785728444*IMU.getAccelX_mss() - 0.05043063791143954;
  ay_cal = 0.9985945019697551*IMU.getAccelY_mss() - -0.15370275713846238;
  az_cal = 0.9827670907542679*IMU.getAccelZ_mss()- -0.951501639184257;

  gx_cal = IMU.getGyroX_rads() - 0.00018500;
  gy_cal = IMU.getGyroY_rads() - -0.00033200;
  gz_cal = IMU.getGyroZ_rads() - 0.00011100;

  mx_off = IMU.getMagX_uT() - 5.844278;
  my_off = IMU.getMagY_uT() - 35.734071;
  mz_off = IMU.getMagZ_uT() - 20.9997;

  mx_cal = 1.105472*(mx_off) + -0.008269*my_off + -0.010815*mz_off;
  my_cal = -0.008269*mx_off + 1.076132*my_off + -0.029695*mz_off;
  mz_cal = -0.010815*mx_off + -0.029695*my_off + 1.214988*mz_off;

  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  fusion.MadgwickUpdate(gx_cal, gy_cal, gz_cal, ax_cal, ay_cal, az_cal, mx_cal, my_cal, mz_cal, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitchRadians();
  roll = fusion.getRollRadians();    //you could also use getRollRadians() ecc
  yaw = convertTo0To2Pi(fusion.getYawRadians());

  while (ss.available() > 0)
    if (gps.encode(ss.read())){
      if (gps.location.isValid())
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
    }
    // get time from GPS module
    if (gps.time.isValid())
    {
        Minute = gps.time.minute();
        Second = gps.time.second();
        Hour   = gps.time.hour();
      }
    // get date drom GPS module
    if (gps.date.isValid())
      {
        Day   = gps.date.day();
        Month = gps.date.month();
        Year  = gps.date.year();
      }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    lcd.print(F("No GPS, check wiring"));
    while(true);
  }


//setTime(Hour, Minute, Second, Day, Month, Year);
//current_t = now();

// Compute the observer's direction in Altitude-Azimuth coordinates
float alt_rad = pitch;
float az_rad = yaw;

// Calculate the observer's declination in radians
float declination_rad = altAzTodec(lat*DEG_TO_RAD, alt_rad, az_rad);

// Calculate the Local Sidereal Time (LST) in hours
float lst_hours = calculateLST(lon, Year, Month, Day, Hour, Minute, Second);

// Calculate the observer's right ascension in radians
float hour_angle_rad = altAzToHA(lat*DEG_TO_RAD, alt_rad, az_rad);
float right_ascension_rad = lst_hours*15*DEG_TO_RAD - hour_angle_rad;

// Convert right ascension and declination from radians to degrees
float right_ascension_deg = right_ascension_rad * RAD_TO_DEG;
float declination_deg = declination_rad * RAD_TO_DEG;

// Wrap angles to the range [0, 360) degrees
right_ascension_deg = fmod(right_ascension_deg, 360.0);
if (right_ascension_deg < 0) {
  right_ascension_deg += 360.0;
}

// Print the calculated celestial coordinates
print_lcd_float(right_ascension_deg, 0, 1);
print_lcd_float(declination_deg, 9, 1);
print_lcd_float(alt_rad*RAD_TO_DEG, 0, 3);
print_lcd_float(az_rad*RAD_TO_DEG, 9, 3);

print_lcd_int(Hour, 17, 1);
print_lcd_int(Minute, 17, 2);
print_lcd_int(Second, 17, 3);


/*Serial.print(lat);
Serial.print(F("\t"));
Serial.print(lon);
Serial.print(F("\t"));
Serial.print(alt_rad*RAD_TO_DEG);
Serial.print(F("\t"));
Serial.print(az_rad*RAD_TO_DEG);
Serial.print(F("\t"));
Serial.print(declination_deg);
Serial.print(F("\t"));
Serial.print(hour_angle_rad*RAD_TO_DEG);
Serial.print(F("\t"));
Serial.print(lst_hours*15);
Serial.print(F("\t"));
Serial.println(right_ascension_deg);*/

delay(5);

}

// Function to convert angle from -π to π to the range of 0 to 2π
float convertTo0To2Pi(float yaw_neg_pi_to_pi) {
  float yaw_0_to_2pi = (yaw_neg_pi_to_pi >= 0) ? yaw_neg_pi_to_pi : (2 * PI + yaw_neg_pi_to_pi);
  return yaw_0_to_2pi;
}

float calculateLST(float longitude_deg, int Year, byte Month, byte Day, byte Hour, byte Minute, byte Second) {
  double JDTT = computeJulianDate(Year, Month, Day, Hour);
  Serial.println(JDTT);
  double JD0 = computeJulianDate(Year, Month, Day, 0.0);
  double DTT = JDTT - 2451545.0;
  double DUT = JD0 - 2451545.0;
  double T = DTT / 36525.0;
  double left = 6.697375 + 0.065707485828*DUT + 1.0027379*Hour + 0.0854103*T + 0.0000258*sq(T);
  double right = 24;
  double GMST = fmod((left + Minute/60.0), right);
  float localSiderealTime = GMST + longitude_deg / 15.0;
  return localSiderealTime ;// Convert to hours
}

// Function to compute the Julian Date for a given date
double computeJulianDate(int year, byte month, byte day, byte UT) {
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

void print_lcd_float(float value, byte row, byte col){
    byte numDigits = 2;
    char buffer[8];
    byte totalWidth;
    lcd.setCursor(row,col);
    totalWidth = snprintf(NULL, 0, "%.*f", numDigits, value) + 1;
    dtostrf(value, totalWidth, 2, buffer);
    lcd.print(buffer);
}

void print_lcd_int(int value, byte row, byte col){
    lcd.setCursor(row,col);
    lcd.print(value);
}
