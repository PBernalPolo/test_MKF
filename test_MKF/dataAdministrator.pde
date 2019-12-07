/*
 * Copyright (C) 2019 Pablo Bernal-Polo
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


// this class manage the data arriving by the serial port
public class dataAdministrator {
  
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
  IPM_MPU6050_HMC5883L agtm;
  IPM_MPU6050 agt;
  IPM_AdafruitIMU9dof amtgt;
  
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
    
    // we initialize the information packets
    this.agtm = new IPM_MPU6050_HMC5883L( (byte)11 );
    this.agt = new IPM_MPU6050( (byte)12 );
    this.amtgt = new IPM_AdafruitIMU9dof( (byte)16 );
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
        this.set_measurementIMU( dataPacket );
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
  private void set_measurementIMU( byte[] b ) {
    // we take the next data array position
    int newCount = this.dataCount + 1;
    if( newCount >= this.Ndat ) newCount = 0;
    // and we update the data
    double[] am = null;
    double[] wm = null;
    switch( b[0] ){  // information packet ID
      case 0:  // IPM_MPU6050_HMC5883L
        this.agt.set_bytes( b );
        am = this.agt.get_a();
        wm = this.agt.get_w();
        break;
      case 1:  // IPM_MPU6050
        this.agtm.set_bytes( b );
        am = this.agtm.get_a();
        wm = this.agtm.get_w();
        break;
      case 3:  // IPM_AdafruitIMU9dof
        this.amtgt.set_bytes( b );
        am = this.amtgt.get_a();
        wm = this.amtgt.get_w();
        break;
      default:
        break;
    }
    if(  am != null  &&  wm != null  ){
      this.amx[newCount] = (float)am[0]*this.amScale + this.ra*randomGaussian();
      this.amy[newCount] = (float)am[1]*this.amScale + this.ra*randomGaussian();
      this.amz[newCount] = (float)am[2]*this.amScale + this.ra*randomGaussian();
      this.wmx[newCount] = (float)wm[0]*this.wmScale + this.rw*randomGaussian();
      this.wmy[newCount] = (float)wm[1]*this.wmScale + this.rw*randomGaussian();
      this.wmz[newCount] = (float)wm[2]*this.wmScale + this.rw*randomGaussian();
      // we update dataCount
      this.dataCount = newCount;
    }
  }
  
  
  // this method is called when a new complete data package arrives by the serial port, and the data source is 1: simulated static dada
  void simulateStaticData(){
    // we take the next data array position
    int newCount = this.dataCount + 1;
    if( newCount >= this.Ndat ) newCount = 0;
    // and we update the data
    this.amx[newCount] = this.ra*randomGaussian();
    this.amy[newCount] = this.ra*randomGaussian();
    this.amz[newCount] = this.ra*randomGaussian() + 1.0;
    this.wmx[newCount] = this.rw*randomGaussian();
    this.wmy[newCount] = this.rw*randomGaussian();
    this.wmz[newCount] = this.rw*randomGaussian();
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
    this.amx[newCount] = this.ra*randomGaussian();
    this.amy[newCount] = this.ra*randomGaussian();
    this.amz[newCount] = this.ra*randomGaussian();
    this.wmx[newCount] = this.rw*randomGaussian();
    this.wmy[newCount] = this.rw*randomGaussian();
    this.wmz[newCount] = this.rw*randomGaussian();
    // we update dataCount
    this.dataCount = newCount;
    
    return;
  }
  
  double[] get_am() {
    return new double[]{ this.amx[this.dataCount] , this.amy[this.dataCount] , this.amz[this.dataCount] };
  }
  
  double[] get_wm() {
    return new double[]{ this.wmx[this.dataCount] , this.wmy[this.dataCount] , this.wmz[this.dataCount] };
  }
  
}