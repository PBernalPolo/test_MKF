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


public abstract class OrientationEstimator {
  
  // PUBLIC METHODS
  
  public OrientationEstimator(){
    this.q[0] = 1.0;   this.q[1] = 0.0;   this.q[2] = 0.0;   this.q[3] = 0.0;
    // it is necessary to set an angular velocity different from 0.0 to break the symmetry
    // otherwise, the MUKF could not converge (especially when we apply the "reset operation" with the RV chart)
    this.w[0] = 1.0e-10;   this.w[1] = 1.0e-10;   this.w[2] = 1.0e-10;
    
    for(int k=0; k<36; k++) this.P[k] = 0.0;
    for(int k=0; k<36; k+=7) this.P[k] = 1.0e2;
    this.P[2+2*6] = 1.0e-16;
    
    for(int k=0; k<9; k++){
      this.Qw[k] = 0.0;
      this.Qa[k] = 0.0;
      this.Rw[k] = 0.0;
      this.Ra[k] = 0.0;
    }
    for(int k=0; k<9; k+=4){
      this.Qw[k] = 1.0e1;
      this.Qa[k] = 1.0e-2;
      this.Rw[k] = 1.0e-3;
      this.Ra[k] = 1.0e-3;
    }
  }
  
  public void get_q( double[] qOut ){
    for(int i=0; i<4; i++) qOut[i] = this.q[i];
  }
  
  public void set_q( double[] qIn ){
    for(int i=0; i<4; i++) this.q[i] = qIn[i];
    for(int k=0; k<36; k++) this.P[k] = 0.0;
    for(int k=0; k<36; k+=7) this.P[k] = 1.0e-8;
    this.P[2+2*6] = 1.0e-16;
  }
  
  public void reset_orientation(){
    this.q[0] = 1.0;   this.q[1] = 0.0;   this.q[2] = 0.0;   this.q[3] = 0.0;
    this.w[0] = 0.0;   this.w[1] = 0.0;   this.w[2] = 0.0;
    
    for(int k=0; k<36; k++) this.P[k] = 0.0;
    for(int k=0; k<36; k+=7) this.P[k] = 1.0e2;
    this.P[2+2*6] = 1.0e-16;
  }
  
  public void set_Qw( double QwIn ){
    for(int k=0; k<9; k++) this.Qw[k] = 0.0;
    for(int k=0; k<9; k+=4) this.Qw[k] = QwIn;
  }
  
  public void set_Qa( double QaIn ){
    for(int k=0; k<9; k++) this.Qa[k] = 0.0;
    for(int k=0; k<9; k+=4) this.Qa[k] = QaIn;
  }
  
  public void set_Rw( double RwIn ){
    for(int k=0; k<9; k++) this.Rw[k] = 0.0;
    for(int k=0; k<9; k+=4) this.Rw[k] = RwIn;
  }
  
  public void set_Ra( double RaIn ){
    for(int k=0; k<9; k++) this.Ra[k] = 0.0;
    for(int k=0; k<9; k+=4) this.Ra[k] = RaIn;
  }
  
  public void set_chartUpdate( boolean chartUpdateIn ){
    this.chartUpdate = chartUpdateIn;
  }
  
  public void set_W0( double W0In ){
    this.W0 = W0In;
  }
  
  // ABSTRACT METHODS
  public abstract void updateIMU( double[] am , double[] wm , double dt );
  
  
  // PRIVATE VARIABLES
  // quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw)
  // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
  protected double[] q = new double[4];
  // angular velocity (rad/s)
  protected double[] w = new double[3];
  // covariance matrix
  protected double[] P = new double[36];
  // covariance matrix of the angular velocity noise (rad^2/s^3)
  protected double[] Qw = new double[9];
  // covariance matrix of the acceleration noise (g^2)
  protected double[] Qa = new double[9];
  // covariance matrix of the angular velocity measurement noise (rad^2/s^2)
  protected double[] Rw = new double[9];
  // covariance matrix of the acceleration measurement noise (g^2)
  protected double[] Ra = new double[9];
  // use or not the chart update
  protected boolean chartUpdate = true;
  // weight of the sigma point produced with the distribution mean (only MUKF)
  double W0 = 1.0/25.0;
  
}