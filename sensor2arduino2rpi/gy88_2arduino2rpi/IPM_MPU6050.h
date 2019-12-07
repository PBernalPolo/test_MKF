
#ifndef INFORMATION_PACKET_MANAGER_MPU6050_H
#define INFORMATION_PACKET_MANAGER_MPU6050_H

#include "IPM_IMU.h"


class IPM_MPU6050 :
      public IPM_IMU {
  
public:
  ///////////////////////////////////////////////////////////////////////////
  // CONSTRUCTORS
  ///////////////////////////////////////////////////////////////////////////
  IPM_MPU6050( int8_t ID );
  
  ///////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////
  void set_a( int16_t ax , int16_t ay , int16_t az );
  void set_w( int16_t wx , int16_t wy , int16_t wz );
  void set_T( int16_t T );
  double* get_a();
  double get_Ta();
  double* get_w();
  double get_Tw();
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  char* toString();
#endif
  
};


//#include "IPM_MPU6050.cpp"


///////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS
///////////////////////////////////////////////////////////////////////////

IPM_MPU6050::IPM_MPU6050( int8_t ID ) {
  this->blength = NON_MEASUREMENT_BYTES+14;  // 7*int16
  this->b = new int8_t[this->blength];
  this->b[0] = 0;  // information packet ID
  this->b[1] = ID;  // sensor ID
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  this->s = new char[50];  // 3*6 + 4 + 3*6 + 4 + 1*6
#endif
}


///////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS
///////////////////////////////////////////////////////////////////////////

void IPM_MPU6050::set_a( int16_t ax , int16_t ay , int16_t az ) {
  IPM::encode_int16( ax , this->b , 2 );
  IPM::encode_int16( ay , this->b , 4 );
  IPM::encode_int16( az , this->b , 6 );
}

void IPM_MPU6050::set_w( int16_t wx , int16_t wy , int16_t wz ) {
  IPM::encode_int16( wx , this->b , 8 );
  IPM::encode_int16( wy , this->b , 10 );
  IPM::encode_int16( wz , this->b , 12 );
}

void IPM_MPU6050::set_T( int16_t T ) {
  IPM::encode_int16( T , this->b , 14 );
}

// IPM_IMU implementation

double* IPM_MPU6050::get_a() {
  this->a[0] = IPM::decode_int16( this->b , 2 );
  this->a[1] = IPM::decode_int16( this->b , 4 );
  this->a[2] = IPM::decode_int16( this->b , 6 );
  return this->a;
}

double IPM_MPU6050::get_Ta() {
  return IPM::decode_int16( this->b , 14 );
}

double* IPM_MPU6050::get_w() {
  this->w[0] = IPM::decode_int16( this->b , 8 );
  this->w[1] = IPM::decode_int16( this->b , 10 );
  this->w[2] = IPM::decode_int16( this->b , 12 );
  return this->w;
}

double IPM_MPU6050::get_Tw() {
  return this->get_Ta();
}

// toString

#if defined INFORMATION_PACKET_MANAGER_TOSTRING
char* IPM_MPU6050::toString() {
  double* am = this->get_a();
  double* wm = this->get_w();
  sprintf( this->s , "%d\t%d\t%d\t\t%d\t%d\t%d\t\t%d" , am[0] , am[1] , am[2] , wm[0] , wm[1] , wm[2] , this->get_Ta() );
  return this->s;
}
#endif

#endif

