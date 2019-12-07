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

//#define DEBUG_MODE
#define DEVICE_ID_AGM 16
#define LED_PIN 13


// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <LSM303DLHC.h>
#include <L3GD20H.h>
#include "IPM_AdafruitIMU9dof.h"
#include "MessageManager.h"


// default address is 105
// specific I2C address may be passed here
LSM303DLHC accelMag;
// specific I2C address may be passed here
L3GD20H gyro;

IPM_AdafruitIMU9dof mAGM( DEVICE_ID_AGM );
MessageManager MM( 2 );

// variables to store the measurements
int16_t ax, ay, az;
int16_t mx, my, mz;
int16_t T;
int16_t wx, wy, wz;
int8_t Tw;

bool blinkState = false;


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  // initialize serial communication
  Serial.begin(115200);
  while( !Serial );
  
  // we leave some time for the sensor to power up
  delay( 300 );
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelMag.initialize();
  accelMag.setAccelLowPowerEnabled( false );  // low power is less update frequency
  accelMag.setAccelOutputDataRate( 1344 );  // Hz
  accelMag.setAccelBlockDataUpdateEnabled( true );  // prevents reading while updating measurements
  accelMag.setAccelFullScale( 16 );  // 16g
  accelMag.setMagOutputDataRate( 220 );  // Hz (max 220)
  accelMag.setMagGain( 230 );  // 8.1Ga
  accelMag.setMagMode( LSM303DLHC_MD_CONTINUOUS );
  accelMag.setMagTemperatureEnabled( true );
  gyro.initialize();
  gyro.setOutputDataRate( 800 );  // Hz (max 800)
  gyro.setBlockDataUpdateEnabled( true );  // prevents reading while updating measurements
  gyro.setFullScale( 2000 );  // 2000deg/s
  
  // we leave some time for the sensor to update the measurements
  delay( 300 );
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelMag.testConnection() ? "LSM303DLHC connection successful" : "LSM303DLHC connection failed");
  Serial.println(gyro.testConnection() ? "L3GD20H connection successful" : "L3GD20H connection failed");
  
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);

  // test
  //test_measurementFrequency();
}


void loop() {
  // read raw accel/mag measurements from device
  accelMag.getAcceleration(&ax, &ay, &az);
  accelMag.getMag(&mx, &my, &mz);
  T = accelMag.getTemperature();
  gyro.getAngularVelocity( &wx , &wy, &wz );
  Tw = gyro.getTemperature();
  // we set the information packet
  mAGM.set_a( ax , ay , az );
  mAGM.set_m( mx , my , mz );
  mAGM.set_T( T );
  mAGM.set_w( wx , wy , wz );
  mAGM.set_Tw( Tw );
  
#if defined DEBUG_MODE
  Serial.print( ax );   Serial.print("\t");   Serial.print( ay );   Serial.print("\t");   Serial.print( az );   Serial.print("\t\t");
  Serial.print( mx );   Serial.print("\t");   Serial.print( my );   Serial.print("\t");   Serial.print( mz );   Serial.print("\t\t");
  Serial.print( T );   Serial.print("\t\t");
  Serial.print( wx );   Serial.print("\t");   Serial.print( wy );   Serial.print("\t");   Serial.print( wz );   Serial.print("\t\t");
  Serial.print( Tw );   Serial.print("\t\t");
  Serial.println();
#else
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( mAGM.get_length() , mAGM.get_bytes() );
  // and we send it
  Serial.write( (char*)toWrite , MM.get_messageOutLength() );
#endif
}


void test_measurementFrequency() {
  int N = 1000;
  float dt0 = 0.0;
  float dt1 = 0.0;
  float dt2 = 0.0;
  float dt3 = 0.0;
  float dt4 = 0.0;
  for(int i=0; i<N; i++){
    unsigned long t0 = micros();
    accelMag.getAcceleration(&ax, &ay, &az);
    unsigned long t1 = micros();
    accelMag.getMag(&mx, &my, &mz);
    unsigned long t2 = micros();
    T = accelMag.getTemperature();
    unsigned long t3 = micros();
    gyro.getAngularVelocity( &wx , &wy, &wz );
    unsigned long t4 = micros();
    Tw = gyro.getTemperature();
    unsigned long t5 = micros();
    dt0 += t1-t0;
    dt1 += t2-t1;
    dt2 += t3-t2;
    dt3 += t4-t3;
    dt4 += t5-t4;
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
  Serial.println();
}

