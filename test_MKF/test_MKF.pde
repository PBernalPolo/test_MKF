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


import processing.serial.*;  // to get serial data


// SERIAL COMMUNICATION VARIABLES
int samplesFromLastUpdate = 0;  // we will use this variable to count the samples per second arriving to the serial port
float sampleFrequency = 0.0;  // this variable will store the last measured sample rate (data rate)

// DRAW METHOD VARIABLES
boolean loopToggle = true;  // loopToggle variable will be used to enable the draw loop or to pause it
int lastMillis;  // this variable will help us to update information at low rate
final int millisPerUpdate = 1000;  // this parameter sets the interval at which low rate information is updated

// ESTIMATORS VARIABLES
final int N_estimators = 9;
long lastResetPositionTime = 0;  // time when last reset position occurred
int updates = 0;  // updates variable will count the frequency at which the OrientationEstimator's will be updated

// the GUI
myGUI theGUI;

// fleet of spacecrafts object
Fleet theFleet = new Fleet();

// data administrator object
dataAdministrator dataAdmin;
CommunicationManager cm;




void setup() {
  // first of all, we set the size of the window (the parameters are set as a function of the width and height; for other different sizes it must be displayed more or less correctly)
  size(1152,864,P3D); //size(800,600,P3D);
  // you can set other parameters, or use the fullScreen mode:
  //fullScreen( P3D , SPAN );  // to EXIT fullScreen mode, press ESC
  
  dataAdmin = new dataAdministrator();
  cm = new CommunicationManager( this , dataAdmin , theFleet );
  
  // we create the GUI
  theGUI = new myGUI( this );
  
  // we create the spacecraft shape
  theFleet.setUpSpacecraft();
  
  // we create the spacecrafts of the fleet. Each spacecraft has an estimator associated. We also specify a position for the orientation mode
  int row = 0;
  int col = 0;
  for(int n=0; n<N_estimators; n++){
    // we define the position
    float[] r0In = { width/(4.0*2.0)*(2.0*col+1) , height/(3.0*2.0)*(2.0*row+1) , 0.0 };
    col++;
    if( col > 3 ){
      col = 0;
      row++;
    }
    // and we add the information to the GUI
    switch( n ){
      case 0:
        theFleet.addSpacecraft( "MUKF O" , new MUKFcO() , r0In );
        break;
      case 1:
        theFleet.addSpacecraft( "MUKF RP" , new MUKFcRP(), r0In );
        break;
      case 2:
        theFleet.addSpacecraft( "MUKF MRP" , new MUKFcMRP() , r0In );
        break;
      case 3:
        theFleet.addSpacecraft( "MUKF RV" , new MUKFcRV() , r0In );
        break;
      case 4:
        theFleet.addSpacecraft( "MEKF O" , new MEKFcO() , r0In );
        break;
      case 5:
        theFleet.addSpacecraft( "MEKF RP" , new MEKFcRP() , r0In );
        break;
      case 6:
        theFleet.addSpacecraft( "MEKF MRP" , new MEKFcMRP() , r0In );
        break;
      case 7:
        theFleet.addSpacecraft( "MEKF RV" , new MEKFcRV() , r0In );
        break;
      case 8:
        theFleet.addSpacecraft( "Madgwick" , new MadgwickAHRS() , r0In );
        break;
    }
  }
  
} // end setup



void draw() {
  cm.fastUpdate_serialPortManagers();
  // we set a black background
  background(0);
  // and the default lights
  lights();
  
  // depending on the scenario (orientation or dead reckoning) we set the perspective and camera
  if( theGUI.scenario == 0 ){
    ortho();
  }else if(theGUI.scenario == 1 ){
    float fov = PI/2.0;
    float cameraZ = (height/2.0) / tan(fov/2.0);
    perspective(fov, float(width)/float(height), cameraZ/100.0, cameraZ*100.0);
    camera(width/2, height/2, cameraZ , width/2, height/2, 0, 0, 1, 0);
  }
  
  // we draw the spacecraft based on the quaternions produced by the estimators
  theFleet.drawFleet();
  
  // if we are in the data tab, we draw the data elements
  if( theGUI.dataToggle.getState() ) theGUI.drawData();
  
  // we reset the position if we have to
  int t = millis();
  if( (t-lastResetPositionTime)*1.0e-3 > 1.0/theGUI.resetPositionFrequency ){
    theGUI.reset_r();
    lastResetPositionTime = t;
  }
  
  // finally, we update the low rate information
  int DeltaT = t-lastMillis;
  if( DeltaT > millisPerUpdate ){
    // sample frequency in the serial port
    sampleFrequency = float(samplesFromLastUpdate)/(1.0e-3*(millis()-lastMillis));
    samplesFromLastUpdate = 0;
    lastMillis = millis();
    
    // real update frequency for our computer
    float measuredUpdateFrequency = updates/(DeltaT/1000.0);
    updates = 0;
    //println( measuredUpdateFrequency );  // this can be used for debug purposes
    // if our computer is not powerful enough to compute the updates at the assigned frequency, we make it lower
    if( theFleet.spacecraft[0].updateFrequency > measuredUpdateFrequency ){
      theGUI.updateFrequencySlider.setValue( (float)Math.log10(measuredUpdateFrequency) );
    }
  }
  
} // end draw



void serialEvent( Serial p ) { 
  cm.notify_activity();
}

void exit() {
  if( cm != null ){
    cm.stop();
  }
  super.exit();
}

// this method is executed each time the mouse is pressed. We use it to pause the draw loop if we click on the visualization part
void mousePressed(){
  boolean mouseInGUI = ( mouseX > theGUI.x0 ) && ( mouseY > theGUI.y0 );
  if( mouseButton == LEFT  &&  !mouseInGUI ){
    if( loopToggle && !mouseInGUI ){
      noLoop();
      loopToggle = false;
    }else{
      loop();
      loopToggle = true;
    }
  }
}