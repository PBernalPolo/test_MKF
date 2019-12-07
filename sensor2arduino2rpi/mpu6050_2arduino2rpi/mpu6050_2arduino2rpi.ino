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
#define DEVICE_ID 14
#define LED_PIN 13


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "IPM_MPU6050.h"
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


IPM_MPU6050 mAG = IPM_MPU6050( DEVICE_ID );
MessageManager MM( 2 );

// variables to store the measurements
int16_t ax, ay, az;
int16_t wx, wy, wz;
int16_t T;

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
  
  // we leave some time for the sensor to update the measurements
  delay( 300 );
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);
  
  // test
  //test_measurementFrequency();
}


void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6( &ax , &ay , &az , &wx , &wy , &wz );
  T = accelgyro.getTemperature();
  // we set the information packet
  mAG.set_a( ax , ay , az );
  mAG.set_w( wx , wy , wz );
  mAG.set_T( T );
  
#if defined DEBUG_MODE
  Serial.print( ax );   Serial.print("\t");   Serial.print( ay );   Serial.print("\t");   Serial.print( az );   Serial.print("\t\t");
  Serial.print( wx );   Serial.print("\t");   Serial.print( wy );   Serial.print("\t");   Serial.print( wz );
  Serial.println();
#else
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( mAG.get_length() , mAG.get_bytes() );
  // and we send it
  Serial.write( (byte*)toWrite , MM.get_messageOutLength() );
#endif
}


void test_measurementFrequency() {
  int N = 1000;
  float dt0 = 0.0;
  float dt1 = 0.0;
  float dt2 = 0.0;
  float dt3 = 0.0;
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
    dt0 += t1-t0;
    dt1 += t2-t1;
    dt2 += t3-t2;
    dt3 += t4-t3;
  }
  Serial.print( 1.0e6/(dt0/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt1/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt2/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt3/N) , 6 );
  Serial.println();
}

