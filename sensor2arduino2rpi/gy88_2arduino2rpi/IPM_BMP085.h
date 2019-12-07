
#ifndef INFORMATION_PACKET_MANAGER_BMP085_H
#define INFORMATION_PACKET_MANAGER_BMP085_H

#include "IPM_Barometer.h"


class IPM_BMP085 :
      public IPM_Barometer {
  
public:
  ///////////////////////////////////////////////////////////////////////////
  // CONSTRUCTORS
  ///////////////////////////////////////////////////////////////////////////
  IPM_BMP085( int8_t ID );
  
  ///////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////
  void set_p( int32_t p );
  void set_T( int16_t T );
  double get_p();
  double get_Tp();
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  char* toString();
#endif
  
};


//#include "IPM_BMP085.cpp"


///////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS
///////////////////////////////////////////////////////////////////////////

IPM_BMP085::IPM_BMP085( int8_t ID ) {
  this->blength = NON_MEASUREMENT_BYTES+6;  // 1*int16 + 1*int32
  this->b = new int8_t[this->blength];
  this->b[0] = 2;  // information packet ID
  this->b[1] = ID;  // sensor ID
#if defined INFORMATION_PACKET_MANAGER_TOSTRING
  this->s = new char[19];  // 11 + 2 + 6
#endif
}


///////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS
///////////////////////////////////////////////////////////////////////////

void IPM_BMP085::set_p( int32_t p ) {
  IPM::encode_int32( p , this->b , 2 );
}

void IPM_BMP085::set_T( int16_t T ) {
  IPM::encode_int16( T , this->b , 6 );
}

// Barometer implementation

double IPM_BMP085::get_p() {
  return IPM::decode_int32( this->b , 2 );
}

double IPM_BMP085::get_Tp() {
  return IPM::decode_int16( this->b , 6 );
}

#if defined INFORMATION_PACKET_MANAGER_TOSTRING
char* IPM_BMP085::toString() {
  sprintf( this->s , "%11d  %6d" , this->P , this->T );
  return this->s;
}
#endif

#endif

