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


// class that implements methods to exchange messages with checksum
public class MessageManager {
  
  // CONSTANTS
  private static final byte magicByte = '\n';  // magic byte to indicate the start of a new package
  private final int Nchecksums;  // number of checksums
  
  // PRIVATE VARIABLES
  private int state;  // state in the management of the incoming message
  private int NRB;  // Number of Received Bytes of the current incoming message
  private byte[] messageIn;  // pointer to array of bytes that will contain the message being received
  private byte[] checksumInM;  // checksum that arrives with the incoming message
  private byte[] checksumInC;  // checksum computed with the incoming message
  private byte[] checksumOut;  // checksum for the outgoing message
  
  
  // CONSTRUCTORS
  
  public MessageManager( int theNchecksums ) {
    if(  theNchecksums <= 0  ||  127 < theNchecksums  ){
      System.out.println( "Not a valid number of checksums. Number of checksums set at 1." );
      theNchecksums = 1;
    }
    this.Nchecksums = (byte)theNchecksums;
    this.checksumOut = new byte[theNchecksums];
    this.checksumInM = new byte[theNchecksums];
    this.checksumInC = new byte[theNchecksums];
    this.state = 0;
    this.NRB = 0;
  }
  
  
  // PUBLIC METHODS
  
  // manages the bytes of the incoming message
  public byte[] manage_byteIn( byte newByte ) {
    byte[] theReturn = null;
    // we act depending on the state of the finite-state machine,
    switch( this.state ){
      case 0:  // we are not receiving a message,
        // we wait for the magic byte
        if( newByte == MessageManager.magicByte ){  // if we find the magic byte,
          this.NRB = 0;  // we initialize the number of received bytes
          this.state = 1;  // and we go to the next state
        }
        break;
      case 1:  // in this state we obtain the message length
        if( newByte > 0 ){  // the length of the message has to be positive
          this.messageIn = new byte[newByte];
          this.state = 2;  // in the next state we will receive the message
        // uncommenting these lines will allow to receive messages of zero length
//        }else if( newByte == 0 ){
//          this.messageIn = new byte[0];
//          this.state = 3;
        }else{
          this.state = 0;  // if newByte is negative, it can not be the length of the array
        }
        break;
      case 2:  // we know the size of the message, and we are receiving it
        this.messageIn[this.NRB++] = newByte;  // we store the received bytes
        // if we complete the message,
        if( this.NRB >= this.messageIn.length ){
          // we go to receive the checksum
          this.NRB = 0;
          this.state = 3;
        }
        break;
      case 3:  // we have finished receiving the message. Now we are receiving the checksum
        this.checksumInM[this.NRB++] = newByte;  // we store the received bytes
        // if we complete the checksum,
        if( this.NRB >= this.Nchecksums ){
          // we check if the message is correct
          this.checksumInC[0] = 1;
          for(int c=1; c<this.Nchecksums; c++) this.checksumInC[c] = 0;
          for(int i=0; i<this.messageIn.length; i++){
            this.checksumInC[0] += this.messageIn[i];
            for(int c=1; c<this.Nchecksums; c++) this.checksumInC[c] += this.checksumInC[c-1];
          }
          // if the checksum is correct,
          boolean correct = true;
          for(int c=0; c<this.Nchecksums; c++){
            if( this.checksumInC[c] != this.checksumInM[c] ){
              correct = false;
              break;
            }
          }
          if( correct ) theReturn = this.messageIn;
          // after receiving the checksum, the next state is the initial state
          this.state = 0;
        }
        break;
      default:
        // if we are in some other state, we go back to the initial one
        this.state = 0;
        break;
    }  // end switch( this.state )
    return theReturn;
  }  // end manage_byteIn( byte newByte )
  
  
  // prepares the bytes of an outgoing message
  public byte[] prepare_message( byte[] theMessage ) {
    if( theMessage.length > 127 ) return null;
    // the prepared message will be: ( magicByte , message.length , (message) , (checksums) )
    byte[] messageOut = new byte[ 2 + theMessage.length + this.Nchecksums ];
    messageOut[0] = MessageManager.magicByte;
    messageOut[1] = (byte)theMessage.length;
    // we compute the checksums
    this.checksumOut[0] = 1;
    for(int c=1; c<this.Nchecksums; c++) this.checksumOut[c] = 0;
    for(int i=0; i<theMessage.length; i++){
      messageOut[2+i] = theMessage[i];
      this.checksumOut[0] += theMessage[i];
      for(int c=1; c<this.Nchecksums; c++) this.checksumOut[c] += this.checksumOut[c-1];
    }
    for(int c=0; c<this.Nchecksums; c++) messageOut[2+theMessage.length+c] = this.checksumOut[c];
    return messageOut;
  }  // end prepare_message( byte[] theMessage )
  
}