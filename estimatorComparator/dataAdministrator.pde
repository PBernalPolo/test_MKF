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

/**
 *
 * @author P.Bernal-Polo
 */


// this class manage the data arriving by the serial port
public class dataAdministrator{
  
  // VARIABLES
  int dataSource;  // data sources are { 0: serial data , 1: simulated static data , 2: simulated bad data }
  int Ndat;  // number of stored data packages (set up in the constructor)
  // acceleration and angular velocity measurements
  float[] amx;
  float[] amy;
  float[] amz;
  float[] wmx;
  float[] wmy;
  float[] wmz;
  int dataCount;  // this variable will be the array position of the last data
  float Tshow;  // time interval of showed data
  float amScale;  // factor used to convert raw acceleration data to data measured in g units
  float wmScale;  // factor used to convert raw gyroscope data to data measured in rad/s
  float ra;  // standard deviation of the normal distributed noise added to the accelerometer measurement
  float rw;  // standard deviation of the normal distributed noise added to the gyroscope measurement
  
  
  // constructor
  dataAdministrator(){
    this.Ndat = 10000;  // we store 10000 by default becasue Tshow is 10.0 by default and the data rate is about 1000.0 samples/s
    this.amx = new float[Ndat];
    this.amy = new float[Ndat];
    this.amz = new float[Ndat];
    this.wmx = new float[Ndat];
    this.wmy = new float[Ndat];
    this.wmz = new float[Ndat];
    for(int k=0; k<this.Ndat; k++){
      this.amx[k] = 0.0;
      this.amy[k] = 0.0;
      this.amz[k] = 0.0;
      this.wmx[k] = 0.0;
      this.wmy[k] = 0.0;
      this.wmz[k] = 0.0;
    }
    this.dataCount = 0;
    this.Tshow = 10.0;
    this.amScale = 16.0/(1<<15);  // the arduino code is configured to get accelerations in the range [ -16.0 , 16.0 ] g
    this.wmScale = 2000.0*PI/180.0/(1<<15);  // the arduino code is configured to get angular velocities in the range [ -2000.0 , 2000.0 ] degrees/s
    
    // initially we do not add noise
    this.ra = 0.0;
    this.rw = 0.0;
  }
  
  
  // sets the data source from the GUI
  void set_dataSource( float theValue ){
    this.dataSource = (int)theValue;
  }
  
  
  // sets the acceleration noise from the GUI
  void set_Ra( double theValue ){
    this.ra = (float)Math.sqrt( theValue );
  }
  
  
  // sets the gyroscope noise from the GUI
  void set_Rw( double theValue ){
    this.rw = (float)Math.sqrt( theValue );
  }
  
  
  // updates the data depending on the data source
  void updateData( byte[] dataPacket ){
    // first we look at the data source
    switch( this.dataSource ){
      case 0:  // serial data
        // we add the package to the measurements
        this.newSerialMeasurements( dataPacket );
        break;
      case 1:  // simulated static data
        // we simulate the static data
        this.simulateStaticData();
        break;
      case 2:  // simulated bad data
        // we simulate the bad data
        this.simulateBadData();
        break;
    }
  }
  
  
  // this method is called when a new complete data package arrives by the serial port, and the data source is 0: serial data
  void newSerialMeasurements( byte[] dataPacket ){
    // we take the next data array position
    int newCount = this.dataCount + 1;
    if( newCount >= this.Ndat ) newCount = 0;
    // and we update the data
    amx[newCount] = ( ( dataPacket[0] << 8 ) | ( dataPacket[1] & 0xFF ) )*amScale + this.ra*randomGaussian();
    amy[newCount] = ( ( dataPacket[2] << 8 ) | ( dataPacket[3] & 0xFF ) )*amScale + this.ra*randomGaussian();
    amz[newCount] = ( ( dataPacket[4] << 8 ) | ( dataPacket[5] & 0xFF ) )*amScale + this.ra*randomGaussian();
    wmx[newCount] = ( ( dataPacket[6] << 8 ) | ( dataPacket[7] & 0xFF ) )*wmScale + this.rw*randomGaussian();
    wmy[newCount] = ( ( dataPacket[8] << 8 ) | ( dataPacket[9] & 0xFF ) )*wmScale + this.rw*randomGaussian();
    wmz[newCount] = ( ( dataPacket[10] << 8 ) | ( dataPacket[11] & 0xFF ) )*wmScale + this.rw*randomGaussian();
    // we update dataCount
    this.dataCount = newCount;
    
    return;
  }
  
  
  // this method is called when a new complete data package arrives by the serial port, and the data source is 1: simulated static dada
  void simulateStaticData(){
    // we take the next data array position
    int newCount = this.dataCount + 1;
    if( newCount >= this.Ndat ) newCount = 0;
    // and we update the data
    amx[newCount] = this.ra*randomGaussian();
    amy[newCount] = this.ra*randomGaussian();
    amz[newCount] = this.ra*randomGaussian() + 1.0;
    wmx[newCount] = this.rw*randomGaussian();
    wmy[newCount] = this.rw*randomGaussian();
    wmz[newCount] = this.rw*randomGaussian();
    // we update dataCount
    this.dataCount = newCount;
    
    return;
  }
  
  
  // this method is called when a new complete data package arrives by the serial port, and the data source is 2: simulated bad data
  void simulateBadData(){
    // we take the next data array position
    int newCount = this.dataCount + 1;
    if( newCount >= this.Ndat ) newCount = 0;
    // and we update the data
    amx[newCount] = this.ra*randomGaussian();
    amy[newCount] = this.ra*randomGaussian();
    amz[newCount] = this.ra*randomGaussian();
    wmx[newCount] = this.rw*randomGaussian();
    wmy[newCount] = this.rw*randomGaussian();
    wmz[newCount] = this.rw*randomGaussian();
    // we update dataCount
    this.dataCount = newCount;
    
    return;
  }
  
}
