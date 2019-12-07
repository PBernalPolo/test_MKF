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


// we import the serial library
import processing.serial.*;


// class that manages connected serial devices
public class CommunicationManager implements Runnable {
  
  // PARAMETERS
  private String[] dontOpen = new String[]{ "/dev/rfcomm0" , "/dev/ttyAMA0" , "/dev/serial1" ,
//                                            "/dev/serial" , "/dev/ttyUSB0" , "/dev/ttyUSB1" , "/dev/ttyUSB2"  // only if we want to read from the xsense and the crossbow
                                          };
  
  // PRIVATE VARIABLES
  private PApplet thePApplet;
  private boolean running;  // true while the thread is running
  private boolean wait;  // true if we want to pause the thread
  private boolean waiting;  // true if we are waiting for activity
  private int stateN;  // state of the finite-state machine that manages the network
  private int NSPM;  // Number of Serial Port Managers in the last update (in the list; not all will be available)
  private int NASPM;  // Number of Available Serial Port Managers in the last update (only those that can be opened; those that do not throw an exception when you try to open them)
  private SerialPortManager[] SPM;
  private dataAdministrator dataAdmin;
  private Fleet theFleet;
  
  
  // CONSTRUCTORS
  
  public CommunicationManager( PApplet aPApplet , dataAdministrator aDataAdmin , Fleet aFleet ) {
    this.thePApplet = aPApplet;
    this.running = true;
    this.wait = false;
    this.waiting = false;
    this.stateN = 0;
    this.NSPM = 0;
    this.NASPM = 0;
    this.dataAdmin = aDataAdmin;
    this.theFleet = aFleet;
    ( new Thread( this ) ).start();
  }
  
  
  // PUBLIC METHODS
  
  public void fastUpdate_serialPortManagers() {
    // first, we check if the number of serial ports has changed
    if( this.NSPM != Serial.list().length ){
      // we wait a bit for the system to update the serial ports
      delay(1000);
      this.update_serialPortManagers();
    }
  }
  
  public synchronized void update_serialPortManagers() {
    // if the number has changed, we redefine the serial ports
    int newNSPM = Serial.list().length;
    SerialPortManager[] auxSPM = new SerialPortManager[newNSPM];
    int newNASPM = 0;
    for(int j=0; j<newNSPM; j++){
      String theName = Serial.list()[j];
      // we check that the serial port is not prohibited (we do not want to mess with the bluetooth)
      if( !this.canWeOpen( theName ) ) continue;
      // we check if the j-th serial port is already opened
      boolean opened = false;
      for(int i=0; i<this.NASPM; i++){
        if( this.SPM[i].is_thisSerialPort( theName ) ){  // if its name is in the list of available serial ports, then it is opened
          auxSPM[newNASPM] = this.SPM[i];
          newNASPM++;
          opened = true;
          break;
        }
      }
      // if it has not been opened, we try to open it (here is where the objects are created)
      if( !opened ){
        try{
          auxSPM[newNASPM] = new SerialPortManager( this.thePApplet , theName );
          newNASPM++;
        }catch( Exception exc ){
        }
      }
    }
    // then, we close the non-used serial ports
    for(int i=0; i<this.NASPM; i++){
      boolean used = false;
      for(int j=0; j<newNASPM; j++){
        if( this.SPM[i].is_thisSerialPort( auxSPM[j].serialPortName ) ){
          used = true;
          break;
        }
      }
      if( !used ){
        this.SPM[i].stop();
      }
    }
    // finally, we perform the redefinition
    this.SPM = new SerialPortManager[newNASPM];
    for(int i=0; i<newNASPM; i++){
      this.SPM[i] = auxSPM[i];
      System.out.println( this.SPM[i].serialPortName );
    }
    System.out.println();
    this.NASPM = newNASPM;
    this.NSPM = newNSPM;
  }  // end update_serialPorts
  
  public void notify_activity() {
    this.wait = false;
    if( this.waiting ){
      synchronized( this ){
        this.notify();
      }
    }
  }
  
  public void run() {
    while( this.running ){
      //
      switch( this.stateN ){
        case 0:  // we do not have a server running
          try{
            this.wait = false;  // we do not want it to stop in this state
            this.stateN = 1;
          }catch( Exception e ){
            e.printStackTrace();
          }
          break;
        case 1:  // we have a server running
          // first, we manage the serial ports
          this.manage_serial();
          break;
        default:
          this.stateN = 0;
          break;
      }
      //
      if( this.wait ){
        synchronized( this ){
          this.waiting = true;
          try{
            this.wait(1000);  // we wait at most 1 second
          }catch( Exception e ){
            e.printStackTrace();
          }
          this.waiting = false;
        }
      }else{
        this.wait = true;
      }
    }  // end while( this.running )
  }  // end public void run()
  
  void stop(){
    this.running = false;
    this.notify_activity();
  }
  
  
  // PRIVATE METHODS
  
  private double mod( double x , double y , double z ) {
    return Math.sqrt( x*x + y*y + z*z );
  }
  
  private void manage_serial() {
    for(int i=0; i<this.NASPM; i++){
      while( this.SPM[i].available() > 0 ){
        byte[] data = this.SPM[i].read();
        if( data != null ){
          samplesFromLastUpdate++;
          long t = System.nanoTime();
          // we update the data
          this.dataAdmin.updateData( data );
          // and the estimators
          theFleet.updateEstimators( t , this.dataAdmin.get_am() , this.dataAdmin.get_wm() );
          updates++;
        }
      }
    }
  }
  
  private boolean canWeOpen( String name ) {
    for(int i=0; i<this.dontOpen.length; i++){
      if( name.equals( this.dontOpen[i] ) ){
        return false;
      }
    }
    return true;
  }
  
}