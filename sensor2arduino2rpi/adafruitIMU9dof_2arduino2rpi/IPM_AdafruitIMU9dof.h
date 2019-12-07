
#ifndef INFORMATION_PACKET_MANAGER_AdafruitIMU9dof_H
#define INFORMATION_PACKET_MANAGER_AdafruitIMU9dof_H

#include "IPM_MARG.h"


class IPM_AdafruitIMU9dof :
      public IPM_MARG {
  
public:
  ///////////////////////////////////////////////////////////////////////////////////////
  // CONSTRUCTORS
  ///////////////////////////////////////////////////////////////////////////////////////
  IPM_AdafruitIMU9dof( int8_t ID );
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  void set_a( int16_t ax , int16_t ay , int16_t az );
  void set_m( int16_t mx , int16_t my , int16_t mz );
  void set_T( int16_t T );
  void set_w( int16_t wx , int16_t wy , int16_t wz );
  void set_Tw( int16_t T );
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


//#include "IPM_A16M16T16G16T8.cpp"


///////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS
///////////////////////////////////////////////////////////////////////////

IPM_AdafruitIMU9dof::IPM_AdafruitIMU9dof( int8_t ID ) {
  this->blength = NON_MEASUREMENT_BYTES+21;  // 10*int16 + 1*int8
  this->b = new int8_t[this->blength];
  this->b[0] = 3;  // information packet ID
  this->b[1] = ID;  // sensor ID
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  this->s = new char[78];  // 3*6 + 4 + 3*6 + 4 + 1*6 + 2 + 3*6 + 4 + 4
#endif
}


///////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS
///////////////////////////////////////////////////////////////////////////

void IPM_AdafruitIMU9dof::set_a( int16_t ax , int16_t ay , int16_t az ) {
  IPM::encode_int16( ax , this->b , 2 );
  IPM::encode_int16( ay , this->b , 4 );
  IPM::encode_int16( az , this->b , 6 );
}

void IPM_AdafruitIMU9dof::set_m( int16_t mx , int16_t my , int16_t mz ) {
  IPM::encode_int16( mx , this->b , 8 );
  IPM::encode_int16( my , this->b , 10 );
  IPM::encode_int16( mz , this->b , 12 );
}

void IPM_AdafruitIMU9dof::set_T( int16_t T ) {
  IPM::encode_int16( T , this->b , 14 );
}

void IPM_AdafruitIMU9dof::set_w( int16_t wx , int16_t wy , int16_t wz ) {
  IPM::encode_int16( wx , this->b , 16 );
  IPM::encode_int16( wy , this->b , 18 );
  IPM::encode_int16( wz , this->b , 20 );
}

void IPM_AdafruitIMU9dof::set_Tw( int16_t T ) {
  IPM::encode_int16( T , this->b , 22 );
}

// MARG implementation

double* IPM_AdafruitIMU9dof::get_a() {
  this->a[0] = IPM::decode_int16( this->b , 2 );
  this->a[1] = IPM::decode_int16( this->b , 4 );
  this->a[2] = IPM::decode_int16( this->b , 6 );
  return this->a;
}

double IPM_AdafruitIMU9dof::get_Ta() {
  return IPM::decode_int16( this->b , 14 );
}

double* IPM_AdafruitIMU9dof::get_w() {
  this->w[0] = IPM::decode_int16( this->b , 16 );
  this->w[1] = IPM::decode_int16( this->b , 18 );
  this->w[2] = IPM::decode_int16( this->b , 20 );
  return this->w;
}

double IPM_AdafruitIMU9dof::get_Tw() {
  return IPM::decode_int16( this->b , 22 );
}

double* IPM_AdafruitIMU9dof::get_m() {
  this->m[0] = IPM::decode_int16( this->b , 8 );
  this->m[1] = IPM::decode_int16( this->b , 10 );
  this->m[2] = IPM::decode_int16( this->b , 12 );
  return this->m;
}

double IPM_AdafruitIMU9dof::get_Tm() {
  return this->get_Ta();
}

#if defined INFORMATION_PACKET_MANAGER_TOSTRING
char* IPM_AdafruitIMU9dof::toString() {
  double* am = this->get_a();
  double* wm = this->get_w();
  double* mm = this->get_m();
  sprintf( this->s , "%6d %6d %6d  %6d %6d %6d  %6d  %6d %6d %6d  %6d" , am[0] , am[1] , am[2] , mm[0] , mm[1] , mm[2] , this->get_Ta() , wm[0] , wm[1] , wm[2] , this->get_Tw() );
  return this->s;
}
#endif

#endif

