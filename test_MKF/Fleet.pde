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


// this class contains each spacecraft, and implements the methods acting on the whole fleet
public class Fleet {
  
  // VARIABLES
  // spacecraft shape (we will use the same shape for every spacecraft. There is no necessity of creating the same shape for every single spacecraft)
  PShape spacecraftShape;
  // number of added spacecraft
  int spacecraftCount = 0;
  // spacecraft objects
  Spacecraft[] spacecraft;
  
  
  
  // METHODS
  
  // sets the parameters for a new spacecraft
  void addSpacecraft( String labelIn , OrientationEstimator estimatorIn , float[] r0In ){
    // we create a new array with space for another Spacecraft
    Spacecraft[] newSpacecraft = new Spacecraft[this.spacecraftCount+1];
    // we make the new references point to the old spacecraft objects
    for(int n=0; n<this.spacecraftCount; n++) newSpacecraft[n] = this.spacecraft[n];
    // and we create a new spacecraft
    newSpacecraft[this.spacecraftCount] = new Spacecraft( labelIn , estimatorIn , r0In );
    // finally we increase the spacecraft counter
    this.spacecraftCount++;
    
    // and we set the reference to the new array
    spacecraft = newSpacecraft;
  }
  
  
  // update the estimator of the whole fleet
  void updateEstimators( long t , double[] am , double[] wm ){
    // for each spacecraft
    for(int n=0; n<this.spacecraftCount; n++){
      // we update its estimator
      this.spacecraft[n].updateEstimator( t , am , wm );
    }
  }
  
  
  // computes the angle axis representation from the quaternion representation of a rotation. We need this to use the processing rotate method
  private float[] quaternion2angleAxis( double[] q ){
    float[] angAxis = new float[4];
    float inorm = sqrt( (float)(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]) );
    if( inorm > 0.0 ){
      angAxis[0] = 2.0*(float)Math.atan2( inorm , q[0] );
      inorm = 1.0/inorm;
      angAxis[1] = (float)q[1]*inorm;
      angAxis[2] = (float)q[2]*inorm;
      angAxis[3] = (float)q[3]*inorm;
    }else{
      angAxis[0] = 0.0;
      angAxis[1] = 1.0;
      angAxis[2] = 0.0;
      angAxis[3] = 0.0;
    }
    
    return angAxis;
  }
  
  
  // draws the whole fleet
  void drawFleet(){
    for(int n=0; n<this.spacecraftCount; n++){
      this.spacecraft[n].drawSpacecraft();
    }
  }
  
  
  // draws the spacecraft shape with an orientation given by q, and a position given by r. It also draws the label
  void drawWith( double[] q , float[] r , String label ){
    // we build the angle-axis
    float[] angAxis = this.quaternion2angleAxis( q );
    
    // we reset the matrix
    this.spacecraftShape.resetMatrix();
    // first we rotate around the x-axis to straighten the spacecraft (because of the different processing axis convention)
    this.spacecraftShape.rotateX( HALF_PI );
    // then we rotate the spacecraft in the opposite direction of the rotation axis:
    // x-processing = our y
    // y-processing = - our z
    // z-processing = our x
    this.spacecraftShape.rotate( angAxis[0] , -angAxis[2] , angAxis[3] , -angAxis[1] );
    // and we apply the z rotation specified in the GUI
    this.spacecraftShape.rotateY( theGUI.zRotAngle );
    // finally we set the position
    this.spacecraftShape.translate( r[0] , r[1] , r[2] );
    
    // and we draw the shape
    shape( this.spacecraftShape );
    
    // we draw the label with white color
    fill(255);
    textAlign(LEFT,CENTER);
    text( label , r[0] , r[1] + 80 , r[2] );
    
    return;
  }
  
  
  // creates the spacecraft shape returning a PShape
  public void setUpSpacecraft(){
    
    // SPACECRAFT PARAMETERS
    final float alpha = height/1500.0;  // scale factor for the spacecraft
    // circular part parameters
    final int N_triangles = 10;    //  N_triangles: number of triangles for the circular part
    final float H = 20.0*alpha;          //  H: height for the circular part
    final float R = 100.0*alpha;         //  R: radius of the circular part
    final color C = color(170,200,170);   //  C: color of the circular part
    // connection 1 parameters
    final float Lc1 = 70.0*alpha;        //  Lc1: length of connection 1: circular part with the main motor
    final float Tc1 = 10.0*alpha;        //  Tc1: thickness of connection 1: circular part with the main motor
    final float angc1 = 0.25*PI;   //  angc1: angle of connection 1: circular part with the main motor
    final color Cc1 = color(100,100,100);   //  Cc1: color of connection 1: circular par with main motor
    // motor 1 parameters
    final float Lm1 = 100.0*alpha;       //  Lm1: length of motor 1: main motor
    final float Tm1 = 30.0*alpha;        //  Tm1: thickness of motor 1: main motor
    final color Cm1 = color(150,180,200);   //  Cm1: color of motor 1: main motor
    // connection 2 parameters
    final float Lc2 = 100.0*alpha;       //  Lc2: length of connections 2: main motor the secondary motors
    final float Tc2 = 6.0*alpha;         //  Tc2: thickness of connections 2: main motor the secondary motors
    final float angc2x = 0.25*PI;  //  angc2x: angle of connections 2 in the x direction
    final float angc2y = 0.45*PI;  //  angc2y: angle of connections 2 in the y direction
    final color Cc2 = color(100,100,100);   //  Cc2: color of connection 2: main motor with secondary motors
    // motor 2 parameters
    final float Lm2 = 100.0*alpha;       //  Lm2: length of motor 2: secondary motors
    final float Tm2 = 12.0*alpha;        //  Tm2: thickness of motor 2: secondary motors
    final color Cm2 = color(255,150,150);   //  Cm2: color of secondary motors
    
    
    // we will create the shape as a group (it will be composed of several parts)
    // the group shape is defined as a set of individual shapes. Lets define them
    
    // top of the circular part
    final PShape circularPartTop = createShape();
    circularPartTop.beginShape(TRIANGLE_FAN);
      circularPartTop.vertex( 0.0 , 0.0 , H );
      for(int i=0; i<N_triangles+1; i++){
        float angle = TWO_PI*i/N_triangles;
        circularPartTop.vertex( R*cos(angle) , R*sin(angle) , 0.0 );
      }
    circularPartTop.endShape();
    circularPartTop.setStroke( color(0) );
    circularPartTop.setFill( C );
    
    // bottom of the circular part
    final PShape circularPartBottom = createShape();
    circularPartBottom.beginShape(TRIANGLE_FAN);
      circularPartBottom.vertex( 0.0 , 0.0 , -H );
      for(int i=0; i<N_triangles+1; i++){
        float angle = TWO_PI*i/N_triangles;
        circularPartBottom.vertex( R*cos(angle) , R*sin(angle) , 0.0 );
      }
    circularPartBottom.endShape();
    circularPartBottom.setStroke( color(0) );
    circularPartBottom.setFill( C );
    
    // connection 1: circular part with the main motor
    final PShape connection1 = createShape( BOX , Lc1 , Tc1 , Tc1 );
    connection1.translate( 0.5*Lc1 , 0.0 , 0.0 );
    connection1.rotateY( angc1 );
    connection1.translate( 0.5*R , 0.0 , 0.0 );
    connection1.setFill( Cc1 );
    
    // motor 1: main motor
    final PShape motor1 = createShape( BOX , Lm1 , Tm1 , Tm1 );
    motor1.rotateY(-angc1);
    //motor1.translate( 0.5*Lc1 , 0.0 , 0.0 );
    motor1.translate( Lc1 , 0.0 , 0.0 );
    //motor1.translate( 0.5*Lc1 , 0.0 , 0.0 );
    motor1.rotateY( angc1 );
    motor1.translate( 0.5*R , 0.0 , 0.0 );
    motor1.setFill( Cm1 );
    
    // first we draw the right part
    // connection2r: main motor with secondary right motor
    final PShape connection2r = createShape( BOX , Lc2 , Tc2 , Tc2 );
    connection2r.translate( 0.5*Lc2 , 0.0 , 0.0 );
    connection2r.rotateY( -angc2y );
    connection2r.rotateX( angc2x );
    connection2r.translate( 0.3*Lm1 , 0.0 , 0.0 );
    connection2r.rotateY(-angc1);
    //connection2r.translate( 0.5*Lc1 , 0.0 , 0.0 );
    connection2r.translate( Lc1 , 0.0 , 0.0 );
    //connection2r.translate( 0.5*Lc1 , 0.0 , 0.0 );
    connection2r.rotateY( angc1 );
    connection2r.translate( 0.5*R , 0.0 , 0.0 );
    connection2r.setFill( Cc2 );
    
    // motor2r: secondary right motor
    final PShape motor2r = createShape( BOX , Lm2 , Tm2 , Tm2 );
    motor2r.translate( 0.2*Lm2 , 0.0 , 0.0 );
    motor2r.rotateY(angc2y);
    //motor2r.translate( 0.5*Lc2 , 0.0 , 0.0 );
    motor2r.translate( Lc2 , 0.0 , 0.0 );
    //motor2r.translate( 0.5*Lc2 , 0.0 , 0.0 );
    motor2r.rotateY( -angc2y );
    motor2r.rotateX( angc2x );
    motor2r.translate( 0.3*Lm1 , 0.0 , 0.0 );
    motor2r.rotateY(-angc1);
    //motor2r.translate( 0.5*Lc1 , 0.0 , 0.0 );
    motor2r.translate( Lc1 , 0.0 , 0.0 );
    //motor2r.translate( 0.5*Lc1 , 0.0 , 0.0 );
    motor2r.rotateY( angc1 );
    motor2r.translate( 0.5*R , 0.0 , 0.0 );
    motor2r.setFill( Cm2 );
    
    // now we draw the part
    // connection2l: main motor with secondary left motor
    final PShape connection2l = createShape( BOX , Lc2 , Tc2 , Tc2 );
    connection2l.translate( 0.5*Lc2 , 0.0 , 0.0 );
    connection2l.rotateY( -angc2y );
    connection2l.rotateX( -angc2x );
    connection2l.translate( 0.3*Lm1 , 0.0 , 0.0 );
    connection2l.rotateY(-angc1);
    //connection2l.translate( 0.5*Lc1 , 0.0 , 0.0 );
    connection2l.translate( Lc1 , 0.0 , 0.0 );
    //connection2l.translate( 0.5*Lc1 , 0.0 , 0.0 );
    connection2l.rotateY( angc1 );
    connection2l.translate( 0.5*R , 0.0 , 0.0 );
    connection2l.setFill( Cc2 );
    
    // motor2l: secondary left motor
    final PShape motor2l = createShape( BOX , Lm2 , Tm2 , Tm2 );
    motor2l.translate( 0.2*Lm2 , 0.0 , 0.0 );
    motor2l.rotateY(angc2y);
    //motor2l.translate( 0.5*Lc2 , 0.0 , 0.0 );
    motor2l.translate( Lc2 , 0.0 , 0.0 );
    //motor2l.translate( 0.5*Lc2 , 0.0 , 0.0 );
    motor2l.rotateY( -angc2y );
    motor2l.rotateX( -angc2x );
    motor2l.translate( 0.3*Lm1 , 0.0 , 0.0 );
    motor2l.rotateY(-angc1);
    //motor2l.translate( 0.5*Lc1 , 0.0 , 0.0 );
    motor2l.translate( Lc1 , 0.0 , 0.0 );
    //motor2l.translate( 0.5*Lc1 , 0.0 , 0.0 );
    motor2l.rotateY( angc1 );
    motor2l.translate( 0.5*R , 0.0 , 0.0 );
    motor2l.setFill( Cm2 );
    
    // now we create the group
    this.spacecraftShape = createShape( GROUP );
    // finally we add the "child" shapes to the parent group
    this.spacecraftShape.addChild( circularPartTop );
    this.spacecraftShape.addChild( circularPartBottom );
    this.spacecraftShape.addChild( connection1 );
    this.spacecraftShape.addChild( motor1 );
    this.spacecraftShape.addChild( connection2r );
    this.spacecraftShape.addChild( motor2r );
    this.spacecraftShape.addChild( connection2l );
    this.spacecraftShape.addChild( motor2l );
    
    return;
  }
  
}