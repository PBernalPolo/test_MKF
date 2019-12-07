
// implemented Information Packets:
// information packet ID - name
// 0 - IPM_MPU6050
// 1 - IPM_MPU6050_HMC5883L
// 2 - IPM_BMP085
// 3 - IPM_AdafruitIMU9dof
// 4 - IPM_t_MPU6050_HMC5883L
// 5 - IPM_t_BMP085


#ifndef INFORMATION_PACKET_MANAGER_H
#define INFORMATION_PACKET_MANAGER_H

//#define INFORMATION_PACKET_MANAGER_TOSTRING

#include <stdint.h>


class IPM {
  
protected:
  ///////////////////////////////////////////////////////////////////////////////////////
  // PARAMETERS
  ///////////////////////////////////////////////////////////////////////////////////////
  static const int NON_MEASUREMENT_BYTES = 2;  // information packet ID + sensor ID

  ///////////////////////////////////////////////////////////////////////////////////////
  // VARIABLES
  ///////////////////////////////////////////////////////////////////////////////////////
  int blength;
  int8_t* b;
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  char* s;
#endif
  
public:
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  void set_bytes( int theLength , int8_t* bytePointer );
  int get_length();
  int8_t* get_bytes();
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  char* toString();
#endif
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC STATIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  static void encode_int8( int8_t value , int8_t* b , int index );
  static int8_t decode_int8( int8_t* b , int index );
  static void encode_int16( int16_t value , int8_t* b , int index );
  static int16_t decode_int16( int8_t* b , int index );
  static void encode_int32( int32_t value , int8_t* b , int index );
  static int32_t decode_int32( int8_t* b , int index );
  static void encode_int64( int64_t value , int8_t* b , int index );
  static int64_t decode_int64( int8_t* b , int index );
  static void encode_float( float value , int8_t* b , int index );
  static float decode_float( int8_t* b , int index );
  
};


//#include "IPM.cpp"


///////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS
///////////////////////////////////////////////////////////////////////////

void IPM::set_bytes( int theLength , int8_t* bytePointer ) {
  delete[] this->b;
  this->blength = theLength;
  this->b = bytePointer;
}

int IPM::get_length() {
  return this->blength;
}

int8_t* IPM::get_bytes() {
  return this->b;
}


///////////////////////////////////////////////////////////////////////////
// PUBLIC STATIC METHODS
///////////////////////////////////////////////////////////////////////////

void IPM::encode_int8( int8_t value , int8_t* b , int index ) {
  b[index] = value;
}

int8_t IPM::decode_int8( int8_t* b , int index ) {
  return b[index];
}

void IPM::encode_int16( int16_t value , int8_t* b , int index ) {
  int8_t* packets = (int8_t*) &value;  // we create a int8_t pointer pointing to the address of the int16
  b[index] = packets[0];
  b[index+1] = packets[1];
}

int16_t IPM::decode_int16( int8_t* b , int index ) {
  int16_t* valuePointer = (int16_t*) &b[index];  // we create a int16 pointer pointing to the first byte
  return *valuePointer;  // we return the value of the int16 pointed by the pointer
}

void IPM::encode_int32( int32_t value , int8_t* b , int index ) {
  int8_t* packets = (int8_t*) &value;  // we create a int8_t pointer pointing to the address of the int32
  b[index] = packets[0];
  b[index+1] = packets[1];
  b[index+2] = packets[2];
  b[index+3] = packets[3];
}

int32_t IPM::decode_int32( int8_t* b , int index ) {
  int32_t* valuePointer = (int32_t*) &b[index];  // we create a int32 pointer pointing to the first byte
  return *valuePointer;  // we return the value of the int32 pointed by the pointer
}

void IPM::encode_int64( int64_t value , int8_t* b , int index ) {
  int8_t* packets = (int8_t*) &value;  // we create a int8_t pointer pointing to the address of the int64
  b[index] = packets[0];
  b[index+1] = packets[1];
  b[index+2] = packets[2];
  b[index+3] = packets[3];
  b[index+4] = packets[4];
  b[index+5] = packets[5];
  b[index+6] = packets[6];
  b[index+7] = packets[7];
}

int64_t IPM::decode_int64( int8_t* b , int index ) {
  int64_t* valuePointer = (int64_t*) &b[index];  // we create a int64 pointer pointing to the first byte
  return *valuePointer;  // we return the value of the int64 pointed by the pointer
}

void IPM::encode_float( float value , int8_t* b , int index ) {
  int8_t* packets = (int8_t*) &value;  // we create a int8_t pointer pointing to the address of the float
  b[index] = packets[0];
  b[index+1] = packets[1];
  b[index+2] = packets[2];
  b[index+3] = packets[3];
}

float IPM::decode_float( int8_t* b , int index ) {
  float* valuePointer = (float*) &b[index];  // we create a float pointer pointing to the first byte
  return *valuePointer;  // we return the value of the float pointed by the pointer
}

#endif

