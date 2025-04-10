// for the GPS
#include <Adafruit_GPS.h>
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

// for the mag
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
Adafruit_LIS3MDL lis3mdl;

#include <Adafruit_LSM6DS3TRC.h>
Adafruit_LSM6DS3TRC lsm6ds3trc;

uint32_t timerGPS = millis();
uint32_t timerMAG = millis();

int IMU_LAST_READ = 0;

bool CALIBRATION = true;
// the serial data need to be injected into 
// https://github.com/PaulStoffregen/MotionCal

float ToDecimalDegrees(float formattedLatLon)
{
  float decDegrees = (float)((int)formattedLatLon / 100);
  float decMinutes = formattedLatLon - (decDegrees * 100);
  float fractDegrees = decMinutes / 60.0;

  return decDegrees + fractDegrees;
}

// These are specific to each IMU and IMU environment - 
// Poor calibration values will prevent AHRS function
float mag_hardiron[]  = { 24.42, -16.87, -3.88 }; // in uTesla
float mag_softiron[]  = { \
0.959, 0.050, -0.038, \
0.050, 1.078, 0.002,  \
-0.038, 0.002, 0.970 }; 
float gyro_zerorate[] = { 0.00, 0.02, 0.01 }; // in Radians/s

#include "quaternionFilters.h"
#define MPS2_TO_G 9.80665

void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println("GPS booting");

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  Serial.println("Booting LIS3MDL");
  
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

    Serial.println("Adafruit LSM6DS3TR-C test!");

  if (!lsm6ds3trc.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  // lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds3trc.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds3trc.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

  // lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds3trc.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds3trc.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  //   if (c) Serial.print(c);
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timerGPS > 1000) {
    timerGPS = millis(); // reset the timer
    
    // Serial.print("\nGPS Time: ");
    
    // if (GPS.hour < 10) { Serial.print('0'); }
    // Serial.print(GPS.hour, DEC); Serial.print(':');
    // if (GPS.minute < 10) { Serial.print('0'); }
    // Serial.print(GPS.minute, DEC); Serial.print(':');
    // if (GPS.seconds < 10) { Serial.print('0'); }
    // Serial.print(GPS.seconds, DEC); Serial.print('.');
    // if (GPS.milliseconds < 10) {
    //   Serial.print("00");
    // } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    //   Serial.print("0");
    // }

    // Serial.println(GPS.milliseconds);
    // Serial.print("Date: ");
    // Serial.print(GPS.day, DEC); Serial.print('/');
    // Serial.print(GPS.month, DEC); Serial.print("/20");
    // Serial.println(GPS.year, DEC);
    // Serial.print("Fix: "); Serial.print((int)GPS.fix);
    // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    if (GPS.fix && CALIBRATION == false) {

      float latDecimal = ToDecimalDegrees(GPS.latitude);
      float lonDecimal = ToDecimalDegrees(GPS.longitude);

      // use this if you want DDMM.MMMM and DDDMM.MMMM
      // Degrees and minutes
      // Serial.print("GPS:");
      // Serial.print(GPS.latitude, 4); 
      // Serial.print(GPS.lat);
      // Serial.print(",");
      // Serial.print(GPS.longitude, 4); 
      // Serial.print(GPS.lon);
      
      // use this is you want decimal degrees
      Serial.print("GPS:");
      Serial.print(latDecimal,6);
      Serial.print(GPS.lat);
      Serial.print(",");
      Serial.print(lonDecimal,6);
      Serial.print(GPS.lon);

      Serial.print(",KN:"); 
      Serial.print(GPS.speed);
      Serial.print(",HEAD:"); 
      Serial.print(GPS.angle);
      Serial.print(",ALT:"); 
      Serial.print(GPS.altitude);
      Serial.print(",SAT:"); 
      Serial.println((int)GPS.satellites);
    }
  }

  // approximately 500 ms or so, update the direction
  if (millis() - timerMAG > 50) {
    timerMAG = millis();
    float imu_dt_s = float(timerMAG - IMU_LAST_READ) / 1000;
    IMU_LAST_READ = timerMAG;

    // Get a new normalized sensor event
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;

    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);

    if (CALIBRATION) {
      float ax_G = accel.acceleration.x / MPS2_TO_G;
      float ay_G = accel.acceleration.y / MPS2_TO_G;
      float az_G = accel.acceleration.z / MPS2_TO_G;

      float gx = gyro.gyro.x;
      float gy = gyro.gyro.y;
      float gz = gyro.gyro.z;
          
      // hard iron cal
      float mx = mag.magnetic.x;
      float my = mag.magnetic.y;
      float mz = mag.magnetic.z;

      Serial.print("Raw:");
      Serial.print(int(ax_G*10000)); Serial.print(",");
      Serial.print(int(ay_G*10000)); Serial.print(",");
      Serial.print(int(az_G*10000)); Serial.print(",");
      Serial.print(int(gx*RAD_TO_DEG)); Serial.print(","); // Deg resolution, that's ok
      Serial.print(int(gy*RAD_TO_DEG)); Serial.print(",");
      Serial.print(int(gz*RAD_TO_DEG)); Serial.print(",");
      Serial.print(int(mx*10)); Serial.print(","); // uT*10
      Serial.print(int(my*10)); Serial.print(",");
      Serial.print(int(mz*10)); Serial.println("");

      // unified data
      Serial.print("Uni:");
      Serial.print(ax_G); Serial.print(",");
      Serial.print(ay_G); Serial.print(",");
      Serial.print(az_G); Serial.print(",");
      Serial.print(gx, 4); Serial.print(",");
      Serial.print(gy, 4); Serial.print(",");
      Serial.print(gz, 4); Serial.print(",");
      Serial.print(mx); Serial.print(",");
      Serial.print(my); Serial.print(",");
      Serial.print(mz); Serial.println("");

      return;
    }

    Serial.print("T:");
    Serial.print(temp.temperature);
    Serial.println("C");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("AX:");
    Serial.print(accel.acceleration.x);
    Serial.print("Y:");
    Serial.print(accel.acceleration.y);
    Serial.print("Z:");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2");

    /* Display the results (rotation is measured in rad/s) */
    Serial.print("GX:");
    Serial.print(gyro.gyro.x);
    Serial.print("Y:");
    Serial.print(gyro.gyro.y);
    Serial.print("Z:");
    Serial.print(gyro.gyro.z);
    Serial.println(" rad/s");

    /* Display the results (magnetic field is measured in uTesla) */
    Serial.print("MAG:X:"); Serial.print(mag.magnetic.x);
    Serial.print("Y:"); Serial.print(mag.magnetic.y); 
    Serial.print("Z:"); Serial.print(mag.magnetic.z); 
    Serial.println("uT");

    Serial.print("DT(s):");
    Serial.println(imu_dt_s);

    float ax_G = accel.acceleration.x / MPS2_TO_G;
    float ay_G = accel.acceleration.y / MPS2_TO_G;
    float az_G = accel.acceleration.z / MPS2_TO_G;

    float gx = gyro.gyro.x + gyro_zerorate[0];
    float gy = gyro.gyro.y + gyro_zerorate[1];
    float gz = gyro.gyro.z + gyro_zerorate[2];
        
    // hard iron cal
    float mx = mag.magnetic.x - mag_hardiron[0];
    float my = mag.magnetic.y - mag_hardiron[1];
    float mz = mag.magnetic.z - mag_hardiron[2];
    
    // soft iron cal
    float mxCal = mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    float myCal = mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    float mzCal = mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];

    MahonyQuaternionUpdate(ax_G, ay_G, az_G, gx, gy, gz, mxCal, myCal, mzCal, imu_dt_s);

    String dataLog = String("DATA:");
    dataLog += String(" AX:") + String(ax_G) + String(",AY:") + String(ay_G) + String(",AZ:") + String(az_G);
    dataLog += String(",GX:") + String(gx) + String(",GY:") + String(gy) + String(",GZ:") + String(gz);
    dataLog += String(",MX:") + String(mxCal) + String(",MY:") + String(myCal) + String(",MZ:") + String(mzCal);

    Serial.println(dataLog);

    // NOTE: Written by Kris Winer
    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    float myIMUyaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                  * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                  * *(getQ()+3));
    float myIMUpitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2)));
    float myIMUroll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3));
  
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(myIMUyaw *= RAD_TO_DEG, 2);
    Serial.print(", ");
    Serial.print(myIMUpitch *= RAD_TO_DEG, 2);
    Serial.print(", ");
    Serial.println(myIMUroll *= RAD_TO_DEG, 2);

    String cardinal;

    float headingRadians = atan2(myCal, mxCal);
    // flipped is correct?

    float headingDegrees = headingRadians * 180 / PI;
    float declinationAngle = 0.0;

    headingDegrees += declinationAngle;

    if (headingDegrees < 0) {
      headingDegrees += 360;
    }

    if (headingDegrees > 348.75 || headingDegrees < 11.25) {
      cardinal = " N";
    }
    else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
      cardinal = " NNE";
    }
    else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
      cardinal = " NE";
    }
    else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
      cardinal = " ENE";
    }
    else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
      cardinal = " E";
    }
    else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
      cardinal = " ESE";
    }
    else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
      cardinal = " SE";
    }
    else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
      cardinal = " SSE";
    }
    else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
      cardinal = " S";
    }
    else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
      cardinal = " SSW";
    }
    else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
      cardinal = " SW";
    }
    else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
      cardinal = " WSW";
    }
    else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
      cardinal = " W";
    }
    else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
      cardinal = " WNW";
    }
    else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
      cardinal = " NW";
    }
    else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
      cardinal = " NNW";
    }

    Serial.print("NON_COMP Heading: ");
    Serial.print(headingDegrees);
    Serial.println(cardinal);

  }
}