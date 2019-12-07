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


// class that implements the communication of the serial port
public class SerialPortManager {
  
  // PRIVATE VARIABLES
  private Serial serialPort;  // the serial port
  private String serialPortName;  // name of the serial port
  private MessageManager MM;  // the one that manages the package reception
  
  
  // CONSTRUCTORS
  
  public SerialPortManager( PApplet thePApplet , String theName ) throws Exception {
    try{
      this.serialPort = new Serial( thePApplet , theName , 115200 );
    }catch( Exception e ){
      throw new Exception("Not able to create the Serial object.");
    }
    this.serialPortName = theName;
    this.MM = new MessageManager( 2 );
  }
  
  
  public boolean is_thisSerialPort( String theName ) {
    return ( this.serialPortName.equals( theName ) );
  }
  
  
  public int available() {
    return this.serialPort.available();
  }
  
  
  public byte[] read() {
    return this.MM.manage_byteIn( (byte)this.serialPort.read() );
  }
  
  
  public void stop() {
    this.serialPort.clear();
    this.serialPort.stop();
  }
  
  
}