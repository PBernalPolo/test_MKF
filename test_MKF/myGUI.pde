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


import controlP5.*;  // the wonderful GUI library for processing


// this class contains all GUI elements and implements the methods to link them with external parameters
public class myGUI {
  
  // VARIABLES
  // the ControlP5 object
  ControlP5 cp5;
  
  // point where the GUI begins
  private float x0 = width*1.0/4.0;
  private float y0 = height*2.0/3.0;
  // size of the GUI
  private float sx = width*3.0/4.0;
  private float sy = height*1.0/3.0;
  
  // GUI ELEMENTS PARAMETERS
  // tabs parameters
  int Ntabs = 4;
  int tabsHeight = (int)(height/30.0);
  int tabsWidth = (int)(this.sx/this.Ntabs);
  float[] tpx = new float[this.Ntabs];  // tab position x
  float tpy;  // tab position y
  // controllers parameters
  int elementsDivisionsX = 3*4;  // the GUI grid has 12 positions in the x axis
  int elementsDivisionsY = 3;  // the GUI grid has 3 positions in the y axis
  float elementsSeparation = height/20.0;  // distance between GUI elements
  float gapSizeX = this.sx/this.elementsDivisionsX;  // x size of a place on the GUI grid
  float gapSizeY = (this.sy-this.tabsHeight - height/30.0)/this.elementsDivisionsY;  // y size of a place on the GUI grid
  float[] cpx = new float[ this.elementsDivisionsX ];  // controllers positions x
  float[] cpy = new float[ this.elementsDivisionsY ];  // controllers positions y
  // normal color
  int foregroundColor;  // slider foreground color
  int backgroundColor;  // slider background color
  // data grids parameters
  int xGridA_divisions = 5;
  int yGridA_divisions = 4;
  float xGridA_i = this.x0 + 0.5*this.elementsSeparation;  // x initial point of the acceleration data grid
  float yGridA_i = this.y0 + 0.5*this.elementsSeparation;  // y initial point of the acceleration data grid
  float xGridA_f = this.x0 + 7.8*this.gapSizeX - 0.5*this.elementsSeparation;  // x final point of the acceleration data grid
  float yGridA_f = this.y0 + 0.5*(this.sy-this.tabsHeight) - 0.5*this.elementsSeparation;  // y final point of the acceleration data grid
  float dataLimA = 2.0;  // limit in the acceleration data grid
  int xGridW_divisions = 5;
  int yGridW_divisions = 4;
  float xGridW_i = this.x0 + 0.5*this.elementsSeparation;  // x initial point of the gyroscope data grid
  float yGridW_i = this.y0 + 0.5*(this.sy-this.tabsHeight);  // y initial point of the gyroscope data grid
  float xGridW_f = this.x0 + 7.8*this.gapSizeX - 0.5*this.elementsSeparation;  // x final point of the gyroscope data grid
  float yGridW_f = this.y0 + (this.sy-this.tabsHeight) - this.elementsSeparation;  // y final point of the gyroscope data grid
  float dataLimW = 500.0*PI/180.0;  // limit in the gyroscope data grid
  
  // PARAMETERS
  //public double[] updateFrequency = new double[N_estimators];
  // quaternion representing the rotation transformation from the vehicle to the outside: qv = q * delta_sv (outside <- vehicle) = (outside <- sensor) * (sensor <- vehicle)
  double[] qv0 = { 1.0 , 0.0 , 0.0 , 0.0 };
  // quaternion representing the rotation transformation from the vehicle to the sensor, or each estimator: qv = q * delta_sv (outside <- vehicle) = (outside <- sensor) * (sensor <- vehicle)
  double[] delta_sv = { 1.0 , 0.0 , 0.0 , 0.0 };
  // mean quaternion computed with all the estimators
  double[] qm = { 1.0 , 0.0 , 0.0 , 0.0 };
  // z rotation angle
  float zRotAngle = 0.0;
  // frequency at which the position is reset
  float resetPositionFrequency = 0.0;
  // scenario: { orientation -> 0 , dead reckoning -> 1 }
  int scenario = 0;
  
  // GUI ELEMENTS
  // tabs
  Toggle orientationToggle;
  Toggle deadReckoningToggle;
  Toggle dataToggle;
  Toggle settingsToggle;
  // orientation controllers  &&  deadReckoning controllers
  ScrollableList dropdownSensorProcessor;
  Slider updateFrequencySlider;
  Toggle chartUpdateToggle;
  Button resetOrientationButton;
  Button setQButton;
  Slider qv0Slider;
  Slider zRotSlider;
  Slider W0Slider;
  // deadReckoning controllers
  Button resetPositionButton;
  Slider resetPositionFrequencySlider;
  // data controllers
  ScrollableList dropdownDataSource;
  // settings controllers
  Slider QaSlider;
  Slider QwSlider;
  Slider RaSlider;
  Slider RwSlider;
  
  // data tab shapes
  PShape gridW;
  PShape gridA;
  
  
  // constructor
  myGUI( PApplet thePApplet ){
    // we create the ControlP5 object
    cp5 = new ControlP5( thePApplet );
    
    // we generate the tabs positions
    for(int i=0; i<Ntabs; i++) this.tpx[i] = this.x0 + i*this.sx/this.Ntabs;
    this.tpy = this.y0 + this.sy - this.tabsHeight;
    
    // we generate the Buttons, that will act like tabs
    orientationToggle = cp5.addToggle("orientation")
                           .setLabel( "orientation" )
                           .setValue(true)
                           .setPosition( this.tpx[0] , this.tpy )
                           .setWidth( this.tabsWidth )
                           .setHeight( this.tabsHeight )
                           .plugTo( this , "set_orientationTab" )
                           ;
    orientationToggle.getCaptionLabel().align( ControlP5.CENTER , ControlP5.CENTER );
    
    deadReckoningToggle = cp5.addToggle("deadReckoning")
                             .setLabel( "dead reckoning" )
                             .setValue(false)
                             .setPosition( this.tpx[1] , this.tpy )
                             .setWidth( this.tabsWidth )
                             .setHeight( this.tabsHeight )
                             .plugTo( this , "set_deadReckoningTab" )
                             ;
    deadReckoningToggle.getCaptionLabel().align( ControlP5.CENTER , ControlP5.CENTER );
    
    dataToggle = cp5.addToggle("data")
                    .setLabel( "data" )
                    .setValue(false)
                    .setPosition( this.tpx[2] , this.tpy )
                    .setWidth( this.tabsWidth )
                    .setHeight( this.tabsHeight )
                    .plugTo( this , "set_dataTab" )
                    ;
    dataToggle.getCaptionLabel().align( ControlP5.CENTER , ControlP5.CENTER );
    
    settingsToggle = cp5.addToggle("settings")
                        .setLabel( "settings" )
                        .setValue(false)
                        .setPosition( this.tpx[3] , this.tpy )
                        .setWidth( this.tabsWidth )
                        .setHeight( this.tabsHeight )
                        .plugTo( this , "set_settingsTab" )
                        ;
    settingsToggle.getCaptionLabel().align( ControlP5.CENTER , ControlP5.CENTER );
    
    // we generate the controllers posible positions
    for(int j=0; j<elementsDivisionsX; j++) cpx[j] = this.x0 + 0.5*this.elementsSeparation + j*gapSizeX;
    for(int i=0; i<elementsDivisionsY; i++) cpy[i] = this.y0 + 0.5*this.elementsSeparation + i*gapSizeY;
    
    // we generate the orientation controllers  &&  deadReckoning controllers
    updateFrequencySlider = cp5.addSlider( "updateFrequency" )
                               .setBroadcast(false)
                               .setLabel( "update frequency" )
                               .setPosition( this.cpx[8] , this.cpy[1] )
                               .setSize( (int)(4.0*this.gapSizeX-this.elementsSeparation) , (int)(this.gapSizeY-this.elementsSeparation) )
                               .setRange(0.0,3.0)
                               .setValue(3.0)
                               .setValueLabel( String.valueOf( 1000.0 ) )
                               .setLock(false)
                               .plugTo( this , "set_updateFrequency" )
                               .setBroadcast(true)
                               ;
    foregroundColor = updateFrequencySlider.getColor().getForeground();
    backgroundColor = updateFrequencySlider.getColor().getBackground();
    updateFrequencySlider.setValueLabel( String.format("%.1f", Math.pow(10.0,updateFrequencySlider.getValue()) ) );
    updateFrequencySlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    chartUpdateToggle = cp5.addToggle( "chartUpdate" )
                           .setLabel("chart update")
                           .setPosition( cpx[8] , cpy[2] )
                           .setSize( (int)(2.0*this.gapSizeX-this.elementsSeparation) , (int)(this.gapSizeY-this.elementsSeparation) )
                           .setValue(true)
                           .plugTo( this , "set_chartUpdate" )
                           ;
    chartUpdateToggle.getCaptionLabel().align( ControlP5.CENTER , ControlP5.CENTER );
    
    resetOrientationButton = cp5.addButton( "resetOrientation" )
                                .setLabel("reset\norientation")
                                .setPosition( cpx[10] , cpy[2] )
                                .setSize( (int)(2.0*this.gapSizeX-elementsSeparation) , (int)(this.gapSizeY-this.elementsSeparation) )
                                .plugTo( this , "reset_orientation" )
                                ;
    resetOrientationButton.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    dropdownSensorProcessor = cp5.addScrollableList( "sensorProcessor" )
                                 .setPosition( this.cpx[8] , this.cpy[0] )
                                 .setSize( (int)(4.0*this.gapSizeX-this.elementsSeparation) , (int)(2.0*this.gapSizeY) )
                                 .setBarHeight( (int)(this.gapSizeY-this.elementsSeparation) ) // height of the principal bar
                                 //.setItemHeight( (int)(this.gapSizeY-this.elementsSeparation) ) // height of the dropdown bars
                                 .addItem("sensor limited",0)
                                 .addItem("processor limited (Arduino)",1)
                                 .setValue(0)
                                 .setType(ScrollableList.DROPDOWN) // currently supported DROPDOWN and LIST
                                 .plugTo( this , "set_sensorProcessor" )
                                 ;
    
    setQButton = cp5.addButton( "setQ" )
                    .setLabel("set q")
                    .setPosition( this.cpx[4] , this.cpy[2] )
                    .setSize( (int)(1.0*this.gapSizeX-0.25*this.elementsSeparation) , (int)(1.0*this.gapSizeY-this.elementsSeparation) )
                    .plugTo( this , "set_q" )
                    ;
    setQButton.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    qv0Slider = cp5.addSlider( "qv0Slider" )
                   .setBroadcast(false)
                   .setLabel("q0 slider")
                   .setRange(0.0,TWO_PI)
                   .setValue(0.0)
                   .setPosition( this.cpx[5] , this.cpy[2] )
                   .setSize( (int)(3.0*this.gapSizeX-this.elementsSeparation) , (int)(1.0*this.gapSizeY-this.elementsSeparation) )
                   .plugTo( this , "set_qv0" )
                   .setBroadcast(true)
                   ;
    qv0Slider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    zRotSlider = cp5.addSlider( "zRotSlider" )
                    .setBroadcast(false)
                    .setLabel("z rotation")
                    .setRange(0.0,TWO_PI)
                    .setValue(0.0)
                    .setPosition( this.cpx[5] , this.cpy[1] )
                    .setSize( (int)(3.0*this.gapSizeX-this.elementsSeparation) , (int)(1.0*this.gapSizeY-this.elementsSeparation) )
                    .plugTo( this , "set_zRotAngle" )
                    .setBroadcast(true)
                    ;
    zRotSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    W0Slider = cp5.addSlider( "W0Slider" )
                  .setBroadcast(false)
                  .setLabel("W0  (MUKF)")
                  .setRange(0.0,1.0)
                  .setValue(1.0/25.0)
                  .setPosition( this.cpx[5] , this.cpy[0] )
                  .setSize( (int)(3.0*this.gapSizeX-this.elementsSeparation) , (int)(1.0*this.gapSizeY-this.elementsSeparation) )
                  .plugTo( this , "set_W0" )
                  .setBroadcast(true)
                  ;
    W0Slider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    
    // we generate the deadReckoning controllers
    resetPositionButton = cp5.addButton( "resetPosition" )
                             .setLabel("reset\nposition")
                             .setPosition( this.cpx[0] , this.cpy[2] )
                             .setSize( (int)(1.0*this.gapSizeX-0.25*this.elementsSeparation) , (int)(1.0*this.gapSizeY-this.elementsSeparation) )
                             .plugTo( this , "reset_r" )
                             ;
    resetPositionButton.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    resetPositionButton.setVisible(false);
    
    resetPositionFrequencySlider = cp5.addSlider( "resetPositionFrequencySlider" )
                                      .setBroadcast(false)
                                      .setLabel("reset position frequency")
                                      .setRange(0.0,2.0)
                                      .setValue(0.0)
                                      .setPosition( this.cpx[1] , this.cpy[2] )
                                      .setSize( (int)(3.0*this.gapSizeX-this.elementsSeparation) , (int)(1.0*this.gapSizeY-this.elementsSeparation) )
                                      .plugTo( this , "set_resetPositionFrequency" )
                                      .setBroadcast(true)
                                      ;
    resetPositionFrequencySlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    resetPositionFrequencySlider.setVisible(false);
    
    // we generate the data controllers
    dropdownDataSource = cp5.addScrollableList( "dataSource" )
                            .setPosition( this.cpx[8]+0.5*this.gapSizeX , this.cpy[2] )
                            .setSize( (int)(3.5*this.gapSizeX-this.elementsSeparation) , (int)(2.0*this.gapSizeY) )
                            .setBarHeight( (int)(this.gapSizeY-this.elementsSeparation) ) // height of the principal bar
                            //.setItemHeight( (int)(this.gapSizeY-this.elementsSeparation) ) // height of the dropdown bars
                            .addItem("serial data",0)
                            .addItem("simulated static data",1)
                            .addItem("simulated bad data",2)
                            .setValue(0)
                            .setType(ScrollableList.DROPDOWN) // currently supported DROPDOWN and LIST
                            .plugTo( dataAdmin , "set_dataSource" )
                            ;
    dropdownDataSource.setVisible(false);
    
    // we generate the settings controllers
    float theValue = -2.0;
    QaSlider = cp5.addSlider( "QaSlider" )
                  .setBroadcast(false)
                  .setLabel( "Qa" )
                  .setRange(-6.0,1.0)
                  .setValue( theValue )
                  .setPosition( this.cpx[7] - 3.0*this.elementsSeparation , this.cpy[0] )
                  .setSize( (int)(1.0*this.gapSizeX-this.elementsSeparation) , (int)(3.0*this.gapSizeY-this.elementsSeparation) )
                  .plugTo( this , "set_Qa" )
                  .setBroadcast(true)
                  ;
    QaSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    double realValue = Math.pow(10.0,theValue);
    QaSlider.setValueLabel( String.format("%4.1e", realValue ) );
    QaSlider.setVisible(false);
    
    theValue = 1.0;
    QwSlider = cp5.addSlider( "QwSlider" )
                  .setBroadcast(false)
                  .setLabel( "Qw" )
                  .setRange(-6.0,1.0)
                  .setValue( theValue )
                  .setPosition( this.cpx[8] - 2.0*this.elementsSeparation , this.cpy[0] )
                  .setSize( (int)(1.0*this.gapSizeX-this.elementsSeparation) , (int)(3.0*this.gapSizeY-this.elementsSeparation) )
                  .plugTo( this , "set_Qw" )
                  .setBroadcast(true)
                  ;
    QwSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    realValue = Math.pow(10.0,theValue);
    QwSlider.setValueLabel( String.format("%4.1e", realValue ) );
    QwSlider.setVisible(false);
    
    theValue = -8.0;
    RaSlider = cp5.addSlider( "RaSlider" )
                  .setBroadcast(false)
                  .setLabel( "Ra" )
                  .setRange(-6.0,1.0)
                  .setValue( theValue )
                  .setPosition( this.cpx[9] - 1.0*this.elementsSeparation , this.cpy[0] )
                  .setSize( (int)(1.0*this.gapSizeX-this.elementsSeparation) , (int)(3.0*this.gapSizeY-this.elementsSeparation) )
                  .plugTo( this , "set_Ra" )
                  .setBroadcast(true)
                  ;
    RaSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    realValue = Math.pow(10.0,theValue);
    RaSlider.setValueLabel( String.format("%4.1e", realValue ) );
    RaSlider.setVisible(false);
    
    theValue = -8.0;
    RwSlider = cp5.addSlider( "RwSlider" )
                  .setBroadcast(false)
                  .setLabel( "Rw" )
                  .setRange(-6.0,1.0)
                  .setValue( theValue )
                  .setPosition( this.cpx[10] , this.cpy[0] )
                  .setSize( (int)(1.0*this.gapSizeX-this.elementsSeparation) , (int)(3.0*this.gapSizeY-this.elementsSeparation) )
                  .plugTo( this , "set_Rw" )
                  .setBroadcast(true)
                  ;
    RwSlider.getCaptionLabel().align( ControlP5.LEFT , ControlP5.BOTTOM_OUTSIDE );
    realValue = Math.pow(10.0,theValue);
    RwSlider.setValueLabel( String.format("%4.1e", realValue ) );
    RwSlider.setVisible(false);
    
    // we set the grids for the data
    this.gridA = generateGrid( this.xGridA_divisions , this.yGridA_divisions , this.xGridA_i , this.yGridA_i , this.xGridA_f , this.yGridA_f );
    this.gridW = generateGrid( this.xGridW_divisions , this.yGridW_divisions , this.xGridW_i , this.yGridW_i , this.xGridW_f , this.yGridW_f );
    
  }
  
  
  // returns true if some tab is active (this help us transforming toggles to tabs)
  boolean someTab(){
    return orientationToggle.getState() || deadReckoningToggle.getState() || dataToggle.getState() || settingsToggle.getState();
  }
  
  
  // implements the behaviour of the orientation tab: activates all controllers in the orientation tab, and deactivates other ones
  void set_orientationTab(){
    if( orientationToggle.getState() ){
      deadReckoningToggle.setState( false );
      dataToggle.setState( false );
      settingsToggle.setState( false );
      scenario = 0;
      dropdownSensorProcessor.setVisible(true);
      updateFrequencySlider.setVisible(true);
      resetOrientationButton.setVisible(true);
      chartUpdateToggle.setVisible(true);
      setQButton.setVisible(true);
      qv0Slider.setVisible(true);
      zRotSlider.setVisible(true);
      W0Slider.setVisible(true);
      resetPositionButton.setVisible(false);
      resetPositionFrequencySlider.setVisible(false);
      dropdownDataSource.setVisible(false);
      QaSlider.setVisible(false);
      QwSlider.setVisible(false);
      RaSlider.setVisible(false);
      RwSlider.setVisible(false);
      for(int n=0; n<theFleet.spacecraftCount; n++) theFleet.spacecraft[n].visibleToggle.setVisible( false );
    }else if( !someTab() ){
      orientationToggle.setState( true );
    }
  }
  
  
  // implements the behaviour of the deadReckoning tab: activates all controllers in the deadReckoning tab, and deactivates other ones
  void set_deadReckoningTab(){
    if( deadReckoningToggle.getState() ){
      orientationToggle.setState( false );
      dataToggle.setState( false );
      settingsToggle.setState( false );
      scenario = 1;
      dropdownSensorProcessor.setVisible(true);
      updateFrequencySlider.setVisible(true);
      resetOrientationButton.setVisible(true);
      chartUpdateToggle.setVisible(true);
      setQButton.setVisible(true);
      qv0Slider.setVisible(true);
      zRotSlider.setVisible(true);
      W0Slider.setVisible(true);
      resetPositionButton.setVisible(true);
      resetPositionFrequencySlider.setVisible(true);
      dropdownDataSource.setVisible(false);
      QaSlider.setVisible(false);
      QwSlider.setVisible(false);
      RaSlider.setVisible(false);
      RwSlider.setVisible(false);
      for(int n=0; n<theFleet.spacecraftCount; n++) theFleet.spacecraft[n].visibleToggle.setVisible( false );
    }else if( !someTab() ){
      deadReckoningToggle.setState( true );
    }
  }
  
  
  // implements the behaviour of the data tab: activates all controllers in the data tab, and deactivates other ones
  void set_dataTab(){
    if( dataToggle.getState() ){
      orientationToggle.setState( false );
      deadReckoningToggle.setState( false );
      settingsToggle.setState( false );
      dropdownSensorProcessor.setVisible(false);
      updateFrequencySlider.setVisible(false);
      resetOrientationButton.setVisible(false);
      chartUpdateToggle.setVisible(false);
      setQButton.setVisible(false);
      qv0Slider.setVisible(false);
      zRotSlider.setVisible(false);
      W0Slider.setVisible(false);
      resetPositionButton.setVisible(false);
      resetPositionFrequencySlider.setVisible(false);
      dropdownDataSource.setVisible(true);
      QaSlider.setVisible(false);
      QwSlider.setVisible(false);
      RaSlider.setVisible(false);
      RwSlider.setVisible(false);
      for(int n=0; n<theFleet.spacecraftCount; n++) theFleet.spacecraft[n].visibleToggle.setVisible( false );
    }else if( !someTab() ){
      dataToggle.setState( true );
    }
  }
  
  
  // implements the behaviour of the settings tab: activates all controllers in the settings tab, and deactivates other ones
  void set_settingsTab(){
    if( settingsToggle.getState() ){
      orientationToggle.setState( false );
      deadReckoningToggle.setState( false );
      dataToggle.setState( false );
      dropdownSensorProcessor.setVisible(false);
      updateFrequencySlider.setVisible(false);
      resetOrientationButton.setVisible(false);
      chartUpdateToggle.setVisible(false);
      setQButton.setVisible(false);
      qv0Slider.setVisible(false);
      zRotSlider.setVisible(false);
      W0Slider.setVisible(false);
      resetPositionButton.setVisible(false);
      resetPositionFrequencySlider.setVisible(false);
      dropdownDataSource.setVisible(false);
      QaSlider.setVisible(true);
      QwSlider.setVisible(true);
      RaSlider.setVisible(true);
      RwSlider.setVisible(true);
      for(int n=0; n<theFleet.spacecraftCount; n++) theFleet.spacecraft[n].visibleToggle.setVisible( true );
    }else if( !someTab() ){
      settingsToggle.setState( true );
    }
  }
  
  
  // implements the behaviour of the dropdown for a sensor limited or processor limited system
  void set_sensorProcessor( int theValue ){
    if( theValue == 0 ){
      updateFrequencySlider.setLock(false);
      set_updateFrequency( this.updateFrequencySlider.getValue() );
      updateFrequencySlider.setColorForeground( color(foregroundColor) );  
      updateFrequencySlider.setColorBackground( color(backgroundColor) );
    }else{
      updateFrequencySlider.setLock(true);
      for(int n=0; n<4; n++) theFleet.spacecraft[n].updateFrequency = 5.0;
      for(int n=4; n<8; n++) theFleet.spacecraft[n].updateFrequency = 25.0;
      theFleet.spacecraft[8].updateFrequency = 400.0;
      updateFrequencySlider.setColorForeground( color(200,200) );
      updateFrequencySlider.setColorBackground( color(100,100) );
    }
  }
  
  
  // implements the update frequency slider behaviour
  void set_updateFrequency( float theValue ){
    double realValue = Math.pow(10.0,theValue);
    this.updateFrequencySlider.setValueLabel( String.format("%.1f", realValue ) );
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].updateFrequency = realValue;
  }
  
  
  // implements the chart update toggle behaviour
  void set_chartUpdate( boolean theValue ){
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.set_chartUpdate( theValue );
  }
  
  
  // implements the reset orientation button behaviour
  void reset_orientation(){
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.reset_orientation();
  }
  
  
  // implements the set q button behaviour
  void set_q(){
    // first we compute the mean quaternion
    for(int i=0; i<4; i++) this.qm[i] = 0.0;
    // we initialize with the first quaternion
    // then we compute the mean quaternion
    for(int n=0; n<theFleet.spacecraftCount; n++){
      if( theFleet.spacecraft[n].visibleToggle.getState() ){
        double[] q = new double[4];
        theFleet.spacecraft[n].estimator.get_q( q );
        if( q[0] < 0.0 ) for(int i=0; i<4; i++) q[i] = -q[i];
        for(int i=0; i<4; i++) this.qm[i] += q[i];
      }
    }
    double inorm = 0.0;
    for(int i=0; i<4; i++) inorm += this.qm[i]*this.qm[i];
    inorm = 1.0/Math.sqrt(inorm);
    for(int i=0; i<4; i++) this.qm[i] *= inorm;
    
    // we set the delta_sv quaternion
    this.delta_sv[0] = this.qm[0]*qv0[0] + this.qm[1]*qv0[1] + this.qm[2]*qv0[2] + this.qm[3]*qv0[3];
    this.delta_sv[1] = this.qm[0]*qv0[1] - qv0[0]*this.qm[1] - this.qm[2]*qv0[3] + this.qm[3]*qv0[2];
    this.delta_sv[2] = this.qm[0]*qv0[2] - qv0[0]*this.qm[2] - this.qm[3]*qv0[1] + this.qm[1]*qv0[3];
    this.delta_sv[3] = this.qm[0]*qv0[3] - qv0[0]*this.qm[3] - this.qm[1]*qv0[2] + this.qm[2]*qv0[1];
    
    // for each estimator, we set the same mean quaternion
    for(int n=0; n<N_estimators; n++){
      theFleet.spacecraft[n].estimator.set_q( this.qm );
    }
  }
  
  
  // implements the set qv0 slider behaviour
  public void set_qv0( float theValue ){
    this.qv0[0] = cos(0.5*theValue);
    this.qv0[1] = 0.0;
    this.qv0[2] = 0.0;
    this.qv0[3] = sin(0.5*theValue);
    
    // we set the delta_sv quaternion
    this.delta_sv[0] = qm[0]*qv0[0] + qm[1]*qv0[1] + qm[2]*qv0[2] + qm[3]*qv0[3];
    this.delta_sv[1] = qm[0]*qv0[1] - qv0[0]*qm[1] - qm[2]*qv0[3] + qm[3]*qv0[2];
    this.delta_sv[2] = qm[0]*qv0[2] - qv0[0]*qm[2] - qm[3]*qv0[1] + qm[1]*qv0[3];
    this.delta_sv[3] = qm[0]*qv0[3] - qv0[0]*qm[3] - qm[1]*qv0[2] + qm[2]*qv0[1];
  }
  
  
  // implements the z rotation angle slider behaviour
  void set_zRotAngle( float theValue ){
    this.zRotAngle = theValue;
  }
  
  
  // implements the W0 slider behaviour
  void set_W0( float theValue ){
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.set_W0( theValue );
  }
  
  
  // implements the reset position button behaviour
  void reset_r(){
    // we reset the velocity (only N_estimators)
    // for each spacecraft in the fleet
    for(int n=0; n<theFleet.spacecraftCount; n++){
      // we reset the velocity
      for(int i=0; i<3; i++) theFleet.spacecraft[n].v[i] = 0.0;
      // and we reset both positions
      for(int is=0; is<2; is++){
        theFleet.spacecraft[n].r[is][0] = width/2.0;
        theFleet.spacecraft[n].r[is][1] = height/2.0;
        theFleet.spacecraft[n].r[is][2] = 0.0;
      }
    }
  }
  
  
  // implements the reset position frequency slider behaviour
  void set_resetPositionFrequency( float theValue ){
    this.resetPositionFrequency = theValue;
  }
  
  
  // implements the set Qa slider behaviour
  void set_Qa( float theValue ){
    double realValue = Math.pow(10.0,theValue);
    this.QaSlider.setValueLabel( String.format("%4.1e", realValue ) );
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.set_Qa( realValue );
  }
  
  
  // implements the set Qw slider behaviour
  void set_Qw( float theValue ){
    double realValue = Math.pow(10.0,theValue);
    this.QwSlider.setValueLabel( String.format("%4.1e", realValue ) );
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.set_Qw( realValue );
  }
  
  
  // implements the set Ra slider behaviour
  void set_Ra( float theValue ){
    double realValue = Math.pow(10.0,theValue);
    this.RaSlider.setValueLabel( String.format("%4.1e", realValue ) );
    dataAdmin.set_Ra( realValue );
    if( this.dropdownDataSource.getValue() == 0 ) realValue += 1.0e-4;  // this is a common noise in the MPU6050 sensor if we have the serial data selected
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.set_Ra( realValue );
  }
  
  
  // implements the set Rw slider behaviour
  void set_Rw( float theValue ){
    double realValue = Math.pow(10.0,theValue);
    this.RwSlider.setValueLabel( String.format("%4.1e", realValue ) );
    dataAdmin.set_Rw( realValue );
    if( this.dropdownDataSource.getValue() == 0 ) realValue += 1.0e-2;  // this is a common noise in the MPU6050 sensor if we have the serial data selected
    for(int n=0; n<N_estimators; n++) theFleet.spacecraft[n].estimator.set_Rw( realValue );
  }
  
  
  // draw the content of the data tab
  void drawData(){
    
    // first we draw the grids
    shape( theGUI.gridA );
    shape( theGUI.gridW );
    
    // we draw the accelerometer data
    float alpha = 240;
    float sizeXA = this.xGridA_f-this.xGridA_i;
    float sizeYA = this.yGridA_f-this.yGridA_i;
    drawData( dataAdmin.amx , this.dataLimA , this.xGridA_i , this.yGridA_i , sizeXA , sizeYA , color(255,0,0,alpha) );
    drawData( dataAdmin.amy , this.dataLimA , this.xGridA_i , this.yGridA_i , sizeXA , sizeYA , color(0,255,0,alpha) );
    drawData( dataAdmin.amz , this.dataLimA , this.xGridA_i , this.yGridA_i , sizeXA , sizeYA , color(30,144,255,alpha) );
    
    // we draw the gyroscope data
    float sizeXW = this.xGridW_f-this.xGridW_i;
    float sizeYW = this.yGridW_f-this.yGridW_i;
    drawData( dataAdmin.wmx , this.dataLimW , this.xGridW_i , this.yGridW_i , sizeXW , sizeYW , color(255,0,0,alpha) );
    drawData( dataAdmin.wmy , this.dataLimW , this.xGridW_i , this.yGridW_i , sizeXW , sizeYW , color(0,255,0,alpha) );
    drawData( dataAdmin.wmz , this.dataLimW , this.xGridW_i , this.yGridW_i , sizeXW , sizeYW , color(30,144,255,alpha) );
    
    // for the next labels we want the text to be centered
    textAlign(CENTER,CENTER);
    
    // we draw the x axis tics
    int theTextSize = (int)(height/60.0);
    textSize( theTextSize );
    float deltaT = dataAdmin.Tshow/xGridA_divisions;
    float T2text = 0;
    for(int i=0; i<this.xGridW_divisions+1; i++){
      text( String.format("%.1f", T2text ) , this.xGridW_f - theTextSize - i*sizeXW/this.xGridW_divisions , this.yGridW_f + theTextSize );
      T2text -= deltaT;
    }
    
    // we draw the y axis tics (accelerometer)
    float deltaA = 2.0*this.dataLimA/this.yGridA_divisions;
    float A2text = this.dataLimA;
    for(int i=0; i<this.yGridA_divisions+1; i++){
      text( String.format("%.1f", A2text ) , this.xGridA_f + 1.5*theTextSize , this.yGridA_i + i*sizeYA/this.yGridA_divisions );
      A2text -= deltaA;
    }
    
    // we draw the y axis tics (gyroscope)
    float deltaW = 2.0*this.dataLimW/this.yGridW_divisions;
    float W2text = this.dataLimW;
    for(int i=0; i<this.yGridW_divisions+1; i++){
      text( String.format("%.1f", W2text ) , this.xGridW_f + 1.5*theTextSize , this.yGridW_i + i*sizeYW/this.yGridW_divisions );
      W2text -= deltaW;
    }
    
    // now we draw the data rate
    textSize( height/50.0 );
    text( "Data rate:" , 0.5*( this.cpx[9] + this.cpx[10] ) , this.cpy[1] );
    text( String.format("%.1f", sampleFrequency ) , this.cpx[11] , this.cpy[1] );
    
    // we draw the y axis label (accelerometer)
    pushMatrix();
    translate( this.xGridA_f + 1.4*this.elementsSeparation , 0.5*(this.yGridA_i+this.yGridA_f) );
    rotate(HALF_PI);
    text("(g)", 0.0 , 0.0 );
    popMatrix();
    
    // we draw the y axis label (gyroscope)
    pushMatrix();
    translate( this.xGridW_f + 1.4*this.elementsSeparation , 0.5*(this.yGridW_i+this.yGridW_f) );
    rotate(HALF_PI);
    text("(rad/s)", 0.0 , 0.0 );
    popMatrix();
    
  }
  
  
  // draws a given set of data points in a specified area
  void drawData( float[] data , float dataLim , float pX0 , float pY0 , float Sx , float Sy , color col ){
    // we set the color
    stroke( col );
    
    // we create and draw the shape
    noFill();
    beginShape();{
      strokeWeight(1.5);
      int samples = int(dataAdmin.Tshow*sampleFrequency);
      float auxX = Sx/samples;
      float auxY1 = pY0 + 0.5*Sy;
      float auxY2 = 0.5*Sy/dataLim;
      int idat = dataAdmin.dataCount;
      for(int i=0; i<samples; i++){
        if( idat < 0 ) idat += dataAdmin.Ndat;
        vertex( pX0+auxX*float(samples-i) , auxY1 - auxY2*data[idat] );  // -data[] because the y-axis of processing points downwards
        idat--;
      }
    }endShape();
    
    return;
  }
  
  
  // generates the PShape that will act like a grid for the data
  PShape generateGrid( int xDivisions , int yDivisions , float xGrid_i , float yGrid_i , float xGrid_f , float yGrid_f ){
    
    int aux = 128;
    color gridColor = color( aux , aux , aux );
    
    PShape grid;
    
    grid = createShape();
    
    grid.setStroke( gridColor );
    
    grid.beginShape(LINES);
      // we set the line width (it is necessary to set it here)
      grid.strokeWeight(0.5);
      // and we generate the grid with lines
      for(int i=0; i<xDivisions+1; i++){
        grid.vertex( xGrid_i + i*(xGrid_f-xGrid_i)/xDivisions , yGrid_i );
        grid.vertex( xGrid_i + i*(xGrid_f-xGrid_i)/xDivisions , yGrid_f );
      }
      for(int i=0; i<yDivisions+1; i++){
        grid.vertex( xGrid_i , yGrid_i + i*(yGrid_f-yGrid_i)/yDivisions );
        grid.vertex( xGrid_f , yGrid_i + i*(yGrid_f-yGrid_i)/yDivisions );
      }
    grid.endShape();
    
    return grid;
  }
  
  
}