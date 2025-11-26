#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "Waveshare_10Dof-D.h"

// ---------- GPS on SoftwareSerial ----------
static const int GPS_RX_PIN = 10;  // GPS TX -> D10
static const int GPS_TX_PIN = 11;  // GPS RX -> D11 (optional)
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// ---------- IMU / Pressure ----------
bool gbSenserConnectState = false;

void setup() {
  Serial.begin(115200);     // USB -> PC / MATLAB
  gpsSerial.begin(9600);    // Air530 NMEA baud
  Wire.begin();

  // Init IMU + pressure using Waveshare API
  IMU_EN_SENSOR_TYPE motionSensorType, pressureType;
  imuInit(&motionSensorType, &pressureType);

  Serial.println("Waveshare 10DOF IMU + GPS");

  if (IMU_EN_SENSOR_TYPE_ICM20948 == motionSensorType) {
    Serial.println("Motion sensor: ICM-20948");
  } else {
    Serial.println("Motion sensor: NULL");
  }

  if (IMU_EN_SENSOR_TYPE_BMP280 == pressureType) {
    Serial.println("Pressure sensor: BMP280");
  } else {
    Serial.println("Pressure sensor: NULL");
  }

  delay(1000);
}

void loop() {
  // --------- Update GPS parser ---------
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // --------- Read IMU & pressure ---------
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  // Convert to nice units
  float roll  = stAngles.fRoll;     // deg
  float pitch = stAngles.fPitch;    // deg
  float yaw   = stAngles.fYaw;      // deg

  float accX = stAccelRawData.s16X;
  float accY = stAccelRawData.s16Y;
  float accZ = stAccelRawData.s16Z;

  float gyroX = stGyroRawData.s16X;
  float gyroY = stGyroRawData.s16Y;
  float gyroZ = stGyroRawData.s16Z;

  float magX = stMagnRawData.s16X;
  float magY = stMagnRawData.s16Y;
  float magZ = stMagnRawData.s16Z;

  float press_hPa = (float)s32PressureVal / 100.0f;      // hPa
  float alt_m     = (float)s32AltitudeVal / 100.0f;      // m
  float temp_C    = (float)s32TemperatureVal / 100.0f;   // °C

  // --------- GPS values ---------
  double lat = NAN, lon = NAN, alt_gps = NAN;

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    alt_gps = gps.altitude.meters();
  }

  // --------- CSV output for MATLAB ---------
  // roll,pitch,yaw,
  Serial.print(roll, 4);  Serial.print(',');
  Serial.print(pitch, 4); Serial.print(',');
  Serial.print(yaw, 4);   Serial.print(',');

  // accX,accY,accZ,
  Serial.print(accX, 4);  Serial.print(',');
  Serial.print(accY, 4);  Serial.print(',');
  Serial.print(accZ, 4);  Serial.print(',');

  // gyroX,gyroY,gyroZ,
  Serial.print(gyroX, 4); Serial.print(',');
  Serial.print(gyroY, 4); Serial.print(',');
  Serial.print(gyroZ, 4); Serial.print(',');

  // magX,magY,magZ,
  Serial.print(magX, 4);  Serial.print(',');
  Serial.print(magY, 4);  Serial.print(',');
  Serial.print(magZ, 4);  Serial.print(',');

  // press_hPa,alt_m,temp_C,
  Serial.print(press_hPa, 2); Serial.print(',');
  Serial.print(alt_m, 2);     Serial.print(',');
  Serial.print(temp_C, 2);    Serial.print(',');

  // lat,lon,alt_gps
  Serial.print(lat, 8);   Serial.print(',');
  Serial.print(lon, 8);   Serial.print(',');
  Serial.println(alt_gps, 2);

  delay(50);  // ~20 Hz output
}
