
#ifndef INFORMATION_PACKET_MANAGER_MPU6050_HMC5883L_H
#define INFORMATION_PACKET_MANAGER_MPU6050_HMC5883L_H

#include "IPM_MARG.h"


class IPM_MPU6050_HMC5883L :
      public IPM_MARG {
  
public:
  ///////////////////////////////////////////////////////////////////////////////////////
  // CONSTRUCTORS
  ///////////////////////////////////////////////////////////////////////////////////////
  IPM_MPU6050_HMC5883L( int8_t ID );
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  void set_a( int16_t ax , int16_t ay , int16_t az );
  void set_w( int16_t wx , int16_t wy , int16_t wz );
  void set_T( int16_t T );
  void set_m( int16_t mx , int16_t my , int16_t mz );
  double* get_a();
  double get_Ta();
  double* get_w();
  double get_Tw();
  double* get_m();
  double get_Tm();
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  char* toString();
#endif
  
};


//#include "IPM_MPU6050_HCM5883L.cpp"


///////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS
///////////////////////////////////////////////////////////////////////////

IPM_MPU6050_HMC5883L::IPM_MPU6050_HMC5883L( int8_t ID ) {
  this->blength = NON_MEASUREMENT_BYTES+20;  // 10*int16
  this->b = new int8_t[this->blength];
  this->b[0] = 1;  // information packet ID
  this->b[1] = ID;  // sensor ID
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  this->s = new char[72];  // 3*6 + 4 + 3*6 + 4 + 1*6 + 2 + 3*6 + 2
#endif
}


///////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS
///////////////////////////////////////////////////////////////////////////

void IPM_MPU6050_HMC5883L::set_a( int16_t ax , int16_t ay , int16_t az ) {
  IPM::encode_int16( ax , this->b , 2 );
  IPM::encode_int16( ay , this->b , 4 );
  IPM::encode_int16( az , this->b , 6 );
}

void IPM_MPU6050_HMC5883L::set_w( int16_t wx , int16_t wy , int16_t wz ) {
  IPM::encode_int16( wx , this->b , 8 );
  IPM::encode_int16( wy , this->b , 10 );
  IPM::encode_int16( wz , this->b , 12 );
}

void IPM_MPU6050_HMC5883L::set_T( int16_t T ) {
  IPM::encode_int16( T , this->b , 14 );
}

void IPM_MPU6050_HMC5883L::set_m( int16_t mx , int16_t my , int16_t mz ) {
  IPM::encode_int16( mx , this->b , 16 );
  IPM::encode_int16( my , this->b , 18 );
  IPM::encode_int16( mz , this->b , 20 );
}

// IPM_MARG implementation

double* IPM_MPU6050_HMC5883L::get_a() {
  this->a[0] = IPM::decode_int16( this->b , 2 );
  this->a[1] = IPM::decode_int16( this->b , 4 );
  this->a[2] = IPM::decode_int16( this->b , 6 );
  return this->a;
}

double IPM_MPU6050_HMC5883L::get_Ta() {
  return IPM::decode_int16( this->b , 14 );
}

double* IPM_MPU6050_HMC5883L::get_w() {
  this->w[0] = IPM::decode_int16( this->b , 8 );
  this->w[1] = IPM::decode_int16( this->b , 10 );
  this->w[2] = IPM::decode_int16( this->b , 12 );
  return this->w;
}

double IPM_MPU6050_HMC5883L::get_Tw() {
  return this->get_Ta();
}

double* IPM_MPU6050_HMC5883L::get_m() {
  this->m[0] = IPM::decode_int16( this->b , 16 );
  this->m[1] = IPM::decode_int16( this->b , 18 );
  this->m[2] = IPM::decode_int16( this->b , 20 );
  return this->m;
}

double IPM_MPU6050_HMC5883L::get_Tm() {
  return this->get_Ta();
}

// toString

#if defined INFORMATION_PACKET_MANAGER_TOSTRING
char* IPM_MPU6050_HMC5883L::toString() {
  double* am = this->get_a();
  double* wm = this->get_w();
  double* mm = this->get_m();
  sprintf( this->s , "%6d %6d %6d  %6d %6d %6d  %6d  %6d %6d %6d" , am[0] , am[1] , am[2] , wm[0] , wm[1] , wm[2] , this->get_Ta() , mm[0] , mm[1] , mm[2] );
  return this->s;
}
#endif

#endif

