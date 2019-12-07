
//#include <stdint.h>

// class that implements methods to exchange messages with checksum
class MessageManager {
  
  // CONSTANTS
  private:
    static const int8_t magicByte = '\n';  // magic byte to indicate the start of a new package
    
  // PRIVATE VARIABLES
    int Nchecksums;  // number of checksums
    int state;  // state in the management of the incoming message
    int NRB;  // Number of Received Bytes of the current incoming message
    int messageInLength;  // length of the incoming message
    int messageOutLength;  // length of the outgoing message
    int8_t* messageIn;  // pointer to array of bytes that will contain the message being received
    int8_t* checksumInM;  // checksum that arrives with the incoming message
    int8_t* checksumInC;  // checksum computed with the incoming message
    int8_t* messageOut;  // pointer to array of bytes that will contain the outgoing message with the MessageManager format
    int8_t* checksumOut;  // checksum for the outgoing message

  
  public:
  // CONSTRUCTORS
    MessageManager( int theNchecksums );
    
    
  // PUBLIC METHODS
    int8_t* manage_byteIn( int8_t newByte );  // manages the bytes of the incoming message
    int get_messageInLength();  // gets the length of the incoming message
    int8_t* prepare_message( int theMessageLength , int8_t* theMessage );  // prepares the bytes of an outgoing message
    int get_messageOutLength();  // gets the length of the prepared message
};


// CONSTRUCTORS
  
MessageManager::MessageManager( int theNchecksums ) {
  if(  theNchecksums <= 0  ||  127 < theNchecksums  ){
    //System.out.println( "Not a valid number of checksums. Number of checksums set at 1." );
    theNchecksums = 1;
  }
  this->Nchecksums = (int8_t)theNchecksums;
  this->checksumOut = new int8_t[theNchecksums];
  this->checksumInM = new int8_t[theNchecksums];
  this->checksumInC = new int8_t[theNchecksums];
  this->state = 0;
  this->NRB = 0;
}
  
  
// PUBLIC METHODS
  
// manages the bytes of the incoming message
int8_t* MessageManager::manage_byteIn( int8_t newByte ) {
  int8_t* theReturn;
  // we act depending on the state of the finite-state machine,
  switch( this->state ){
    case 0:  // we are not receiving a message,
      // we wait for the magic byte
      if( newByte == MessageManager::magicByte ){  // if we find the magic byte,
        this->NRB = 0;  // we initialize the number of received bytes
        this->state = 1;  // and we go to the next state
      }
      break;
    case 1:  // in this state we obtain the message length
      if( newByte > 0 ){  // the length of the message has to be positive
        this->messageInLength = newByte;
        delete[] this->messageIn;
        this->messageIn = new int8_t[this->messageInLength];
        this->state = 2;  // in the next state we will receive the message
        // uncommenting these lines will allow to receive messages of zero length
//      }else if( newByte == 0 ){
//        this->messageIn = new int8_t[0];
//        this->state = 3;
      }else{
        this->state = 0;  // if newByte is negative, it can not be the length of the array
      }
      break;
    case 2:  // we know the size of the message, and we are receiving it
      this->messageIn[this->NRB++] = newByte;  // we store the received bytes
      // if we complete the message,
      if( this->NRB >= this->messageInLength ){
        // we go to receive the checksum
        this->NRB = 0;
        this->state = 3;
      }
      break;
    case 3:  // we have finished receiving the message. Now we are receiving the checksum
      this->checksumInM[this->NRB++] = newByte;  // we store the received bytes
      // if we complete the checksum,
      if( this->NRB >= this->Nchecksums ){
        // we check if the message is correct
        this->checksumInC[0] = 1;
        for(int c=1; c<this->Nchecksums; c++) this->checksumInC[c] = 0;
        for(int i=0; i<this->messageInLength; i++){
          this->checksumInC[0] += this->messageIn[i];
          for(int c=1; c<this->Nchecksums; c++) this->checksumInC[c] += this->checksumInC[c-1];
        }
        // if the checksum is correct,
        boolean correct = true;
        for(int c=0; c<this->Nchecksums; c++){
          if( this->checksumInC[c] != this->checksumInM[c] ){
            correct = false;
            break;
          }
        }
        if( correct ) theReturn = this->messageIn;
        // after receiving the checksum, the next state is the initial state
        this->state = 0;
      }
      break;
    default:
      // if we are in some other state, we go back to the initial one
      this->state = 0;
      break;
  }  // end switch( this.state )
  return theReturn;
}  // end manage_byteIn( int8_t newByte )


// gets the length of the incoming message
int MessageManager::get_messageInLength() {
  return this->messageInLength;
}


// prepares the bytes of an outgoing message
int8_t* MessageManager::prepare_message( int theMessageLength , int8_t* theMessage ) {
  if( theMessageLength > 127 ) return NULL;
  // the prepared message will be: ( magicByte , message.length , (message) , (checksums) )
  this->messageOutLength = 2 + theMessageLength + this->Nchecksums;
  delete[] this->messageOut;
  this->messageOut = new int8_t[ this->messageOutLength ];
  this->messageOut[0] = MessageManager::magicByte;
  this->messageOut[1] = (int8_t)theMessageLength;
  // we compute the checksums
  this->checksumOut[0] = 1;
  for(int c=1; c<this->Nchecksums; c++) this->checksumOut[c] = 0;
  for(int i=0; i<theMessageLength; i++){
    this->messageOut[2+i] = theMessage[i];
    this->checksumOut[0] += theMessage[i];
    for(int c=1; c<this->Nchecksums; c++) this->checksumOut[c] += this->checksumOut[c-1];
  }
  for(int c=0; c<this->Nchecksums; c++) this->messageOut[2+theMessageLength+c] = this->checksumOut[c];
  return this->messageOut;
}  // end prepare_message( int MessageLength , int8_t* theMessage )


// gets the length of the prepared message
int MessageManager::get_messageOutLength() {
  return this->messageOutLength;
}

