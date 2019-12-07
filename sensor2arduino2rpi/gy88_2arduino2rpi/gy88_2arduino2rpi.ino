/*
 * Copyright (C) 2017 P.Bernal-Polo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// TO CHANGE THE OPTIMIZATION LEVEL, edit:
// /opt/arduino-1.8.5/hardware/arduino/avr/platform.txt
// and substitute:
// compiler.c.flags=-c -g -Os -w -ffunction-sections -fdata-sections -MMD
// compiler.cpp.flags=-c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD
// by:
// compiler.c.flags=-c -g -O3 -w -ffunction-sections -fdata-sections -MMD
// compiler.cpp.flags=-c -g -O3 -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD


//#define DEBUG_MODE
#define DEVICE_ID_AGM 11
#define DEVICE_ID_P 101
//#define PRESSURE_HIGH_RESOLUTION
#define LED_PIN 13
#define IMU_MEASUREMENTS_PER_PRESSURE_MEASUREMENTS 40//8  // c*( 1/558 + 1/1766 + 1/486 ) = ( 1/166 + 1/36 )


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "IPM_MPU6050_HMC5883L.h"
#include "IPM_BMP085.h"
#include "MessageManager.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;

// class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 barometer;

IPM_MPU6050_HMC5883L mAGM( DEVICE_ID_AGM );
IPM_BMP085 mP( DEVICE_ID_P );
MessageManager MM( 2 );

// variables to store the measurements
int16_t ax, ay, az;
int16_t wx, wy, wz;
int16_t T;
int16_t mx, my, mz;
int32_t P;
int16_t Tp;

int cm;
bool blinkState = false;


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  // initialize serial communication
  Serial.begin(115200);
  while( !Serial );
  
  // we leave some time for the sensor to power up
  delay( 300 );
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setRate( 8 );  // while ODR_w=8kHz, ODR_a=1kHz => we divide by 8 to obtain the same sample rates and the same accel measurement is not used twice // no divider -> max sample rate
  accelgyro.setDLPFMode( MPU6050_DLPF_BW_256 );  // no low-pass filter -> max sample rate
  accelgyro.setDHPFMode( MPU6050_DHPF_RESET );  // no high-pass filter
  accelgyro.setFullScaleAccelRange( MPU6050_ACCEL_FS_16 );  // 16g
  accelgyro.setFullScaleGyroRange( MPU6050_GYRO_FS_2000 );  // 2000deg/s
  mag.initialize();
  mag.setDataRate( HMC5883L_RATE_75 );
  mag.setSampleAveraging( HMC5883L_AVERAGING_1 );
  mag.setGain( HMC5883L_GAIN_220 );  // 8.1Ga
  mag.setMode( HMC5883L_MODE_CONTINUOUS );
  barometer.initialize();
  
  // we leave some time for the sensor to update the measurements
  delay( 300 );
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
  
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);
  
  // reset count measurements
  cm = 0;
  
  // test
  //test_measurementFrequency();
}


void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6( &ax , &ay , &az , &wx , &wy , &wz );
  T = accelgyro.getTemperature();
  mag.getHeading( &mx , &my , &mz );
  // we set the information packet
  mAGM.set_a( ax , ay , az );
  mAGM.set_w( wx , wy , wz );
  mAGM.set_T( T );
  mAGM.set_m( mx , my , mz );
  cm++;
  if( cm >= IMU_MEASUREMENTS_PER_PRESSURE_MEASUREMENTS ){
    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
#if defined PRESSURE_HIGH_RESOLUTION
    barometer.setControl(BMP085_MODE_PRESSURE_3);  // 25.5ms conversion time
#else
    barometer.setControl(BMP085_MODE_PRESSURE_0);  // 4.5ms conversion time
#endif
    P = barometer.getRawPressure();
    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    Tp = barometer.getRawTemperature();
    // we set the information packet
    mP.set_p( P );
    mP.set_T( Tp );
  }

#if defined DEBUG_MODE
  Serial.print( ax );   Serial.print("\t");   Serial.print( ay );   Serial.print("\t");   Serial.print( az );   Serial.print("\t\t");
  Serial.print( wx );   Serial.print("\t");   Serial.print( wy );   Serial.print("\t");   Serial.print( wz );   Serial.print("\t\t");
  Serial.print( mx );   Serial.print("\t");   Serial.print( my );   Serial.print("\t");   Serial.print( mz );
  Serial.println();
  Serial.print( P );   Serial.print("\t");   Serial.println( Tp );
#else
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( mAGM.get_length() , mAGM.get_bytes() );
  // and we send it
  Serial.write( (byte*)toWrite , MM.get_messageOutLength() );
  if( cm >= IMU_MEASUREMENTS_PER_PRESSURE_MEASUREMENTS ){
    // we prepare the message
    int8_t* toWriteP = MM.prepare_message( mP.get_length() , mP.get_bytes() );
    // we write the message
    Serial.write( (byte*)toWriteP , MM.get_messageOutLength() );
    cm = 0;
  }
#endif
}


void test_measurementFrequency() {
  int N = 1000;
  float dt0 = 0.0;
  float dt1 = 0.0;
  float dt2 = 0.0;
  float dt3 = 0.0;
  float dt4 = 0.0;
  float dt5 = 0.0;
  float dt6 = 0.0;
  for(int i=0; i<N; i++){
    unsigned long t0 = micros();
    accelgyro.getMotion6( &ax , &ay , &az , &wx , &wy , &wz );
    unsigned long t1 = micros();
    accelgyro.getAcceleration( &ax , &ay , &az );
    unsigned long t2 = micros();
    accelgyro.getRotation( &wx , &wy , &wz );
    unsigned long t3 = micros();
    T = accelgyro.getTemperature();
    unsigned long t4 = micros();
    mag.getHeading( &mx , &my , &mz );
    unsigned long t5 = micros();
#if defined PRESSURE_HIGH_RESOLUTION
    barometer.setControl(BMP085_MODE_PRESSURE_3);  // 25.5ms conversion time
#else
    barometer.setControl(BMP085_MODE_PRESSURE_0);  // 4.5ms conversion time
#endif
    P = barometer.getRawPressure();
    unsigned long t6 = micros();
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    Tp = barometer.getRawTemperature();
    unsigned long t7 = micros();
    dt0 += t1-t0;
    dt1 += t2-t1;
    dt2 += t3-t2;
    dt3 += t4-t3;
    dt4 += t5-t4;
    dt5 += t6-t5;
    dt6 += t7-t6;
  }
  Serial.print( 1.0e6/(dt0/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt1/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt2/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt3/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt4/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt5/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt6/N) , 6 );
  Serial.println();
}

