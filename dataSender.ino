#include "I2Cdev.h"
#include "MPU6050.h"
#include <avr/wdt.h>  // for using the watchdog

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// we will save measurements here
int16_t amx, amy, amz;
int16_t wmx, wmy, wmz;



void setup() {
  // first of all, we set up the watchdog (sometimes the sensor initialization gets stuck. This will prevent it from happening)
  watchdogSetup();
  
  // we start the serial connection
  Serial.begin(115200);

  // we start the connection with the sensor
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  TWBR = 12;
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // set wrange
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}


void loop() {
  // we reset the watchdog timer
  wdt_reset();
  // (if something gets stuck, and the watchdog is not reset, the microcontroller will perform a full system restart)
  
  // we take measurements
  accelgyro.getMotion6( &amx , &amy , &amz , &wmx , &wmy , &wmz );
  
  // we send the package ( each packet is a 8-bit signed two's complement integer )
  Serial.write( (int8_t)(amx >> 8) );  Serial.write( (int8_t)amx );
  Serial.write( (int8_t)(amy >> 8) );  Serial.write( (int8_t)amy );
  Serial.write( (int8_t)(amz >> 8) );  Serial.write( (int8_t)amz );
  Serial.write( (int8_t)(wmx >> 8) );  Serial.write( (int8_t)wmx );
  Serial.write( (int8_t)(wmy >> 8) );  Serial.write( (int8_t)wmy );
  Serial.write( (int8_t)(wmz >> 8) );  Serial.write( (int8_t)wmz );
  // finally we send a newline to help the listener to synchronize
  Serial.write('\n');
  
}



// thanks to Simon Tushev: https://tushev.org/articles/arduino/5/arduino-and-watchdog-timer
void watchdogSetup(void){
  // we disable the interrupts so that configuration is never disrupted and left unfinished
  cli();
  // resets the watchdog timer. This is not always necessary but you certainly do not want your
  // watchdog timing out and resetting while you are trying to set it.
  wdt_reset();
  
  // Threshold value   Constant name   Supported on
  //   15 ms             WDTO_15MS      ATMega 8, 168, 328, 1280, 2560
  //   30 ms             WDTO_30MS      ATMega 8, 168, 328, 1280, 2560
  //   60 ms             WDTO_60MS      ATMega 8, 168, 328, 1280, 2560
  //  120 ms             WDTO_120MS     ATMega 8, 168, 328, 1280, 2560
  //  250 ms             WDTO_250MS     ATMega 8, 168, 328, 1280, 2560
  //  500 ms             WDTO_500MS     ATMega 8, 168, 328, 1280, 2560
  //    1 s              WDTO_1S        ATMega 8, 168, 328, 1280, 2560
  //    2 s              WDTO_2S        ATMega 8, 168, 328, 1280, 2560
  //    4 s              WDTO_4S        ATMega 168, 328, 1280, 2560
  //    8 s              WDTO_8S        ATMega 168, 328, 1280, 2560
  wdt_enable( WDTO_2S );
  
  // Now that we are done with the setup you can re-enable interrupts by typing "sei()".
  // The watchdog includes interrupts so we have to be sure to re-enable this.
  sei();
}

