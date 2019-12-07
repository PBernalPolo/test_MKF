/*
 * 
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


// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
public class MadgwickAHRS extends OrientationEstimator {
  
  // Method: updateIMU
  // implementation of the abstract method "updateIMU" defined in the OrientationEstimator class.
  // This is an interface between the "updateIMU" from the OrientationEstimator class, and the
  // "updateIMU" implemented by Madgwick
  @Override
  public void updateIMU( double[] am , double[] wm , double dt ){
    double gx = wm[0]*180.0/Math.PI;
    double gy = wm[1]*180.0/Math.PI;
    double gz = wm[2]*180.0/Math.PI;
    double ax = am[0];
    double ay = am[1];
    double az = am[2];
    invSampleFreq = dt;
    this.q0 = this.q[0];
    this.q1 = this.q[1];
    this.q2 = this.q[2];
    this.q3 = this.q[3];
    
    this.updateIMU( gx , gy , gz , ax , ay , az );
    
    this.q[0] = this.q0;
    this.q[1] = this.q1;
    this.q[2] = this.q2;
    this.q[3] = this.q3;
    
    return;
  }
  
  
  
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // MADGWICK C++ IMPLEMENTATION (written in java)
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  
  public MadgwickAHRS(){
    super();
	  this.beta = 0.1;
	  this.q0 = 1.0;
	  this.q1 = 0.0;
	  this.q2 = 0.0;
	  this.q3 = 0.0;
	  this.invSampleFreq = 1.0/512.0;
  }
  
  public MadgwickAHRS( double sampleFreqDef , double betaDef ){
    super();
	  this.beta = betaDef;
	  this.q0 = 1.0;
	  this.q1 = 0.0;
	  this.q2 = 0.0;
	  this.q3 = 0.0;
	  this.invSampleFreq = 1.0 / sampleFreqDef;
  }
  
  public void begin( double sampleFrequency ){
    invSampleFreq = 1.0f / sampleFrequency;
  }  
  
  // Method: update
  // method used to update the quaternion with measurements from a gyroscope, an accelerometer, and a magnetometer
  // inputs:
  //  gx, gy, gz: gyroscope readings (degrees/s)
  //  ax, ay, az: accelerometer readings (g)
  //  mx, my, mz: magnetometer readings (?)
  // outputs:
  public void update( double gx , double gy , double gz , double ax , double ay , double az , double mx , double my , double mz ){
	  double recipNorm;
	  double s0, s1, s2, s3;
	  double qDot1, qDot2, qDot3, qDot4;
	  double hx, hy;
	  double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    
	  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		  this.updateIMU( gx , gy , gz , ax , ay , az );
		  return;
	  }
    
	  // Convert gyroscope degrees/sec to radians/sec
	  gx *= 0.0174533f;
	  gy *= 0.0174533f;
	  gz *= 0.0174533f;
    
	  // Rate of change of quaternion from gyroscope
	  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
	  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      
		  // Normalise accelerometer measurement
		  recipNorm = 1.0f/Math.sqrt( ax * ax + ay * ay + az * az );
		  ax *= recipNorm;
		  ay *= recipNorm;
		  az *= recipNorm;
      
		  // Normalise magnetometer measurement
		  recipNorm = 1.0f/Math.sqrt( mx * mx + my * my + mz * mz );
		  mx *= recipNorm;
		  my *= recipNorm;
		  mz *= recipNorm;
      
		  // Auxiliary variables to avoid repeated arithmetic
	    _2q0mx = 2.0f * q0 * mx;
	    _2q0my = 2.0f * q0 * my;
	    _2q0mz = 2.0f * q0 * mz;
	    _2q1mx = 2.0f * q1 * mx;
	    _2q0 = 2.0f * q0;
	    _2q1 = 2.0f * q1;
	    _2q2 = 2.0f * q2;
	    _2q3 = 2.0f * q3;
	    _2q0q2 = 2.0f * q0 * q2;
	    _2q2q3 = 2.0f * q2 * q3;
	    q0q0 = q0 * q0;
		  q0q1 = q0 * q1;
		  q0q2 = q0 * q2;
		  q0q3 = q0 * q3;
		  q1q1 = q1 * q1;
		  q1q2 = q1 * q2;
		  q1q3 = q1 * q3;
		  q2q2 = q2 * q2;
		  q2q3 = q2 * q3;
		  q3q3 = q3 * q3;
      
		  // Reference direction of Earth's magnetic field
		  hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		  hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		  _2bx = Math.sqrt(hx * hx + hy * hy);
		  _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		  _4bx = 2.0f * _2bx;
  		_4bz = 2.0f * _2bz;
      
		  // Gradient decent algorithm corrective step
	  	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		  recipNorm = 1.0/Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		  s0 *= recipNorm;
		  s1 *= recipNorm;
		  s2 *= recipNorm;
		  s3 *= recipNorm;
      
		  // Apply feedback step
		  qDot1 -= beta * s0;
		  qDot2 -= beta * s1;
		  qDot3 -= beta * s2;
		  qDot4 -= beta * s3;
	  }
    
	  // Integrate rate of change of quaternion to yield quaternion
	  q0 += qDot1 * invSampleFreq;
	  q1 += qDot2 * invSampleFreq;
	  q2 += qDot3 * invSampleFreq;
	  q3 += qDot4 * invSampleFreq;
    
  	// Normalise quaternion
	  recipNorm = 1.0/Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	  q0 *= recipNorm;
	  q1 *= recipNorm;
	  q2 *= recipNorm;
	  q3 *= recipNorm;
	  
    return;
  }
  
  //-------------------------------------------------------------------------------------------
  // IMU algorithm update
  // Method: updateIMU
  // method used to update the state information through an IMU measurement
  // inputs:
  //  gx, gy, gz: gyroscope readings (degrees/sec)
  //  ax, ay, az: accelerometer readings (g)
  // outputs:
  public void updateIMU( double gx , double gy , double gz , double ax , double ay , double az ) {
	  double recipNorm;
	  double s0, s1, s2, s3;
	  double qDot1, qDot2, qDot3, qDot4;
	  double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
	  // Convert gyroscope degrees/sec to radians/sec
	  gx *= 0.0174533f;
	  gy *= 0.0174533f;
	  gz *= 0.0174533f;
    
	  // Rate of change of quaternion from gyroscope
	  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
	  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      
		  // Normalise accelerometer measurement
		  recipNorm = 1.0/Math.sqrt(ax * ax + ay * ay + az * az);
		  ax *= recipNorm;
		  ay *= recipNorm;
		  az *= recipNorm;
      
		  // Auxiliary variables to avoid repeated arithmetic
		  _2q0 = 2.0f * q0;
		  _2q1 = 2.0f * q1;
		  _2q2 = 2.0f * q2;
		  _2q3 = 2.0f * q3;
		  _4q0 = 4.0f * q0;
		  _4q1 = 4.0f * q1;
		  _4q2 = 4.0f * q2;
		  _8q1 = 8.0f * q1;
		  _8q2 = 8.0f * q2;
		  q0q0 = q0 * q0;
		  q1q1 = q1 * q1;
		  q2q2 = q2 * q2;
		  q3q3 = q3 * q3;
      
		  // Gradient decent algorithm corrective step
		  s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		  s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		  s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		  s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		  recipNorm = 1.0/Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		  s0 *= recipNorm;
		  s1 *= recipNorm;
		  s2 *= recipNorm;
		  s3 *= recipNorm;
      
		  // Apply feedback step
		  qDot1 -= beta * s0;
		  qDot2 -= beta * s1;
		  qDot3 -= beta * s2;
		  qDot4 -= beta * s3;
	  }
    
	  // Integrate rate of change of quaternion to yield quaternion
	  q0 += qDot1 * invSampleFreq;
	  q1 += qDot2 * invSampleFreq;
	  q2 += qDot3 * invSampleFreq;
	  q3 += qDot4 * invSampleFreq;
    
	  // Normalise quaternion
	  recipNorm = 1.0/Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  	q0 *= recipNorm;
	  q1 *= recipNorm;
	  q2 *= recipNorm;
	  q3 *= recipNorm;
    
    return;
  }
  
  
  // PRIVATE VARIABLES
  private double beta;				// algorithm gain
  private double q0;
  private double q1;
  private double q2;
  private double q3;	// quaternion of sensor frame relative to auxiliary frame
  private double invSampleFreq;
  
}