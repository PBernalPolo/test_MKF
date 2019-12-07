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


// this class implements the methods referents to a single spacecraft
class Spacecraft {
  
  // VARIABLES
  // estimator
  OrientationEstimator estimator;
  // time when last update occurred
  long lastUpdateTime;
  // estimator update frequency for this spacecraft
  double updateFrequency;
  // label to show when drawn
  String label;
  // saved quaternions (there are 2 for each estimator to avoid reading when they are being updated)
  double[][] q = new double[2][4];
  int iq = 0;
  // position in orientation mode
  float[] r0 = { 0.0 , 0.0 , 0.0 };
  // velocity (dead reckoning)
  float[] v = new float[3];
  // position (dead reckoning). There are two to avoid reading while writing
  float[][] r = new float[2][3];
  // index of the last computed position (dead reckoning)
  int ir = 0;
  // boolean flag for visibility
  boolean visible;
  // toggle for visibility
  Toggle visibleToggle;
  
  
  // METHODS
  
  // constructor
  Spacecraft( String labelIn , OrientationEstimator estimatorIn , float[] r0In ){
    this.estimator = estimatorIn;
    this.lastUpdateTime = System.nanoTime();
    this.updateFrequency = 1000.0;
    this.label = labelIn;
    for(int iq=0; iq<2; iq++){
      q[iq][0] = 1.0;
      for(int i=1; i<4; i++) q[iq][i] = 0.0;
    }
    this.iq = 0;
    this.r0 = r0In;
    for(int i=0; i<3; i++) this.v[i] = 0.0;
    for(int ir=0; ir<2; ir++){
      this.r[ir][0] = width/2.0;
      this.r[ir][1] = height/2.0;
      this.r[ir][2] = 0.0;
    }
    this.ir = 0;
    this.visible = true;
    
    // this is a trick to set the visible toggle positions
    float alphaX = 0.25;
    float alphaY = 0.25;
    this.visibleToggle = theGUI.cp5.addToggle( labelIn + "_toggle" )
                               .setLabel( labelIn )
                               .setValue(true)
                               .setPosition( theGUI.x0 + alphaX*r0In[0] , theGUI.y0 + alphaY*r0In[1] )
                               .setVisible( false )
                               .plugTo( this , "set_visible" )
                               ;
    this.visibleToggle.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
  }
  
  
  // implements the behaviour of the visibleToggle
  void set_visible( boolean theValue ){
    this.visible = theValue;
  }
  
  
  // performs an update of the estimator associated with this spacecraft
  void updateEstimator( long t , double[] am , double[] wm ){
    double dt = (t-this.lastUpdateTime)*1.0e-9;
    
    // only if its toggle is on, and if dt = 1/f has passed
    if(  this.visible  &&  dt > 1.0/this.updateFrequency  ){
      // we update the estimator
      this.estimator.updateIMU( am , wm , dt );
      this.lastUpdateTime = t;
      // we store the quaternion each time in a place to avoid reading while writing
      int index;
      if( this.iq == 0 ){
        index = 1;
      }else{
        index = 0;
      }
      this.estimator.get_q( this.q[index] );
      this.iq = index;
      // and if we are in the dead reckoning scenario, we integrate the velocity and position
      if( theGUI.scenario == 1 ){
        // rotation from the first inertial reference frame to the second inertial reference frame (a rotation in the z-axis)
        double[] qz = { cos( 0.5*theGUI.zRotAngle ) , 0.0 , 0.0 , sin( 0.5*theGUI.zRotAngle ) };
        // rotation from the sensor reference frame to the second inertial reference frame
        double[] q = this.q[index]; // { this.q[index][0] , this.spacecraft[n].q[index][1] , this.spacecraft[n].q[index][2] , this.spacecraft[n].q[index][3] };
        double[] qo2s = { qz[0]*q[0] - qz[1]*q[1] - qz[2]*q[2] - qz[3]*q[3] ,
                          qz[0]*q[1] + q[0]*qz[1] + qz[2]*q[3] - qz[3]*q[2] ,
                          qz[0]*q[2] + q[0]*qz[2] + qz[3]*q[1] - qz[1]*q[3] ,
                          qz[0]*q[3] + q[0]*qz[3] + qz[1]*q[2] - qz[2]*q[1] };
        this.integrate( qo2s , am , dt );
      }
    }  
  }
  
  
  // integrates the position of a spacecraft having its estimated orientation, the acceleration measured in the sensor reference frame, and the time step
  void integrate( double[] q , double[] am , double dt ){
    // with this we avoid a strange behaviour in processing: a not undoable deformation on the PShape is produced when the shapes are drawn at far distances
    double distanceFromOrigin = 0.0;
    for(int i=0; i<3; i++) distanceFromOrigin += this.r[0][i]*this.r[0][i];
    distanceFromOrigin = Math.sqrt( distanceFromOrigin );
    if( distanceFromOrigin < 1.0e5 ){
      // we build the rotation matrix
      double[] R = new double[9];
      R[0] = -q[2]*q[2]-q[3]*q[3];    R[3] = q[1]*q[2]-q[3]*q[0];     R[6] = q[1]*q[3]+q[2]*q[0];
      R[1] = q[1]*q[2]+q[3]*q[0];     R[4] = -q[1]*q[1]-q[3]*q[3];    R[7] = q[2]*q[3]-q[1]*q[0];
      R[2] = q[1]*q[3]-q[2]*q[0];     R[5] = q[2]*q[3]+q[1]*q[0];     R[8] = -q[1]*q[1]-q[2]*q[2];
      
      R[0] += R[0] + 1.0;    R[3] += R[3];          R[6] += R[6];
      R[1] += R[1];          R[4] += R[4] + 1.0;    R[7] += R[7];
      R[2] += R[2];          R[5] += R[5];          R[8] += R[8] + 1.0;
      
      // we transform the acceleration from the sensor reference frame to the extern inertial reference frame
      double a[] = new double[3];
      for(int i=0; i<3; i++){
        double sum = 0.0;
        for(int j=0; j<3; j++) sum += R[i+j*3]*am[j];
        a[i] = sum;
      }
      // and we subtract the gravity
      a[2] += -1.0;
      
      // the acceleration is multiplied by a factor in order of make distinguishable the movements produced by the produced accelerations
      // this has the same effect that multiplying the resulting position by the same factor
      float mult = 1.0e4;
      for(int i=0; i<3; i++) a[i] *= mult;
      
      // we update in a different index each time to avoid reading while writing
      int index;
      int index0;
      if( this.ir == 0 ){
        index = 1;
        index0 = 0;
      }else{
        index = 0;
        index0 = 1;
      }
      
      // we integrate velocity and acceleration to obtain position
      this.r[index][0] = this.r[index0][0] + (float)(this.v[0]*dt + 0.5*a[1]*dt*dt);
      this.r[index][1] = this.r[index0][1] + (float)(this.v[1]*dt + 0.5*(-a[2])*dt*dt);
      this.r[index][2] = this.r[index0][2] + (float)(this.v[2]*dt + 0.5*a[0]*dt*dt);
      
      // we integrate the acceleration to obtain velocity
      this.v[0] += a[1]*dt;
      this.v[1] += (-a[2])*dt;
      this.v[2] += a[0]*dt;
      
      // and we update the index
      this.ir = index;
    }
  }
  
  
  // draws this spacecraft
  void drawSpacecraft(){
    // we only draw if the toggle is on
    if( this.visible ){
      // first we check for NaNs
      if( this.areThereNaNs() ){
        this.visibleToggle.setState( false );
        this.estimator.reset_orientation();
        return;
      }
      // if there are no NaNs we compute the orientation from the vehicle ( qv = qs * delta_sv )
      double[] q = this.q[this.iq];
      double[] qv = { q[0]*theGUI.delta_sv[0] - q[1]*theGUI.delta_sv[1] - q[2]*theGUI.delta_sv[2] - q[3]*theGUI.delta_sv[3] ,
                      q[0]*theGUI.delta_sv[1] + theGUI.delta_sv[0]*q[1] + q[2]*theGUI.delta_sv[3] - q[3]*theGUI.delta_sv[2] ,
                      q[0]*theGUI.delta_sv[2] + theGUI.delta_sv[0]*q[2] + q[3]*theGUI.delta_sv[1] - q[1]*theGUI.delta_sv[3] ,
                      q[0]*theGUI.delta_sv[3] + theGUI.delta_sv[0]*q[3] + q[1]*theGUI.delta_sv[2] - q[2]*theGUI.delta_sv[1] };
      // and depending on the scenario (orientation or dead reckoning) we draw the spacecraft
      if( theGUI.scenario == 0 ){
        theFleet.drawWith( qv , this.r0 , this.label );
      }else if( theGUI.scenario == 1 ){
        theFleet.drawWith( qv , this.r[this.ir] , this.label );
      }
    }
    
    return;
  }
  
  
  // it checks if there are NaNs in orientation or position
  boolean areThereNaNs(){
    boolean isNaN = false;
    for(int i=0; i<4; i++) isNaN = isNaN || ( this.q[this.iq][i] != this.q[this.iq][i] );
    if( theGUI.scenario == 1 ) for(int i=0; i<3; i++) isNaN = isNaN || ( this.r[this.ir] != this.r[this.ir] );
    
    return isNaN;
  }
  
}