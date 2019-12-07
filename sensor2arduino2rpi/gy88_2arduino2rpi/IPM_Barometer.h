
#ifndef INFORMATION_PACKET_MANAGER_Barometer_H
#define INFORMATION_PACKET_MANAGER_Barometer_H

#include "IPM.h"


class IPM_Barometer :
      public IPM {
  
public:
  ///////////////////////////////////////////////////////////////////////////////////////
  // ABSTRACT METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  virtual double get_p() = 0;
  virtual double get_Tp() = 0;
  
};

#endif

