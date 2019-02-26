#include <AltSoftSerial.h>
AltSoftSerial midiSerial;

/* MIDI messages consist of a status byte, followed by zero or more data bytes,
 * except for System Exclusive messages, which also have terminating status byte.
   Status bytes always begin with a 1, and data bytes always begin with a 0.
   MIDI messages can be broken up into a few categories:
   
     * Channel Messages: these start with a status byte < 0xF0
       -- These include Note On, Note Off, and Pitch Bend messages.
       -- Their length can be determined from their status byte
       
     * System Exclusive Messages: these start with a status byte = 0xF0
       -- These are variable length messages, specific to manufacturers.
       -- They are terminated by a special status byte, 0xF7, which means
          "End of System Exclusive Mode", roughly.
           
     * Non-Exclusive System Messages: these start with a status byte 
       strictly between 0xF0 and 0xF7 
       -- These include Song Select and MIDI Time Code Messages
       -- Their length can be determined from their status byte
        
     * System Realtime Messages (with a status byte > 0xF7
       -- These include Timing Clock messages
       -- All of these messages are single byte long (so only a status byte) 
   
   We can think of status bytes as "headers" of MIDI messages. But more accurately,
   they can be thought of as requesting a change in the mode/status of the MIDI
   device receiving the signal. A status byte tells the device to enter the right
   mode/status to interpret whatever bytes might come next correctly. */
     


///////////////////////////////////////////////
// Functions for Detecting Type of MIDI Byte //
///////////////////////////////////////////////


// Gives number of data bytes of a non-realtime message given its status byte (header).
// NOTE: should only be called on status bytes (headers) for non-realtime messages.

uint8_t numDataBytes(uint8_t headerInQuestion) { 
  switch (headerInQuestion >> 4) {           // handles channel status bytes
    case 0x08:
      return 2;
    case 0x09:
      return 2;
    case 0x0A:
      return 2;
    case 0x0B:
      return 2;
    case 0x0C:
      return 1;
    case 0x0D:
      return 1;
    case 0x0E:
      return 2;
  }
  switch (headerInQuestion & 0x0F) {         // handles system status bytes 
    case 0x00:    // System Exclusive Start Byte. We return 0, which means
      return 0;   // we ignore system exclusive messages entirely. 
    case 0x01:
      return 1;
    case 0x02:
      return 2;
    case 0x03:
      return 1;
    case 0x04:    // undefned MIDI status, so we ignore it
      return 0; 
    case 0x05:    // undefined MIDI status, so we ignore it
      return 0;
    case 0x06:
      return 0;
    case 0x07:    // System Exclusive Termination Status
      return 0; 
  }
  return 0;    
} 



// Returns true iff a a byte is a header (status byte).

bool isHeader(uint8_t byteInQuestion) { 
  if ( (byteInQuestion >> 7) == 0 ) {
    return false; 
  } else {
    return true;
  }
}



// Returns true if and only if a byte is a realtime message. Realtime messages
// are a single byte long, so the status byte is the entire message.

bool isRealtimeMessage(uint8_t byteInQuestion) {
  if ( (byteInQuestion >> 3) == 0x1F ) {
    return true;
  } else {
    return false;
  }
}


// For status bytes, return true iff it is a channel header.
// Should only be called on status bytes (headers).

bool isChannelMessage(uint8_t headerInQuestion) { 
  if ( headerInQuestion < 0xF0 ) {
    return true;
  } else {
    return false;
  }
} 


//////////////////////////////////////
// Structures for Storing MIDI Data //
//////////////////////////////////////


// Structure for non-system exclusive messages.

struct _message {
  uint8_t header;
  uint8_t data[2];
  uint8_t numDataBytes;
  bool channelMessage;     // indicates whether this is a channel message
};



// Circular buffer structure for messages.
 
struct _message_buffer {
  _message message[32];
  uint8_t head = 0;             // head is location of next message to write to
  uint8_t tail = 0;             // tail is location of next ungotten message
  uint8_t freeSpots = 32;
  void put(_message& messageToPut) {   // passes by reference to save resources
    message[head] = messageToPut;      // should only be called if buffer not full
    head = (head + 1) % 32;
    freeSpots--;
  }
  _message get() {              // should only be called if buffer not empty
    freeSpots++;
    uint8_t oldtail = tail;
    tail = (tail + 1) % 32;
    return message[oldtail];    
  }
  _message peek() {
    return message[tail];    
  }
  bool notEmpty() {
    return (freeSpots < 32);
  }
  bool notFull() {
    return (freeSpots > 0);
  }
  void add(uint8_t header, uint8_t dataByte1, uint8_t dataByte2) { // should only be called if
    message[head].header = header;                                 // buffer is not full
    message[head].numDataBytes = numDataBytes(header);
    message[head].channelMessage = isChannelMessage(header);
    message[head].data[0] = dataByte1;
    message[head].data[1] = dataByte2;
    head = (head + 1) % 32;
    freeSpots--;
  }
};


// Circular buffer structure for bytes.

struct _byte_buffer {
  uint8_t bytes[32];
  uint8_t head = 0; 
  uint8_t tail = 0;  
  uint8_t freeBytes = 32;
  void put(uint8_t byteToPut) {   // should only be called if buffer not full
    bytes[head] = byteToPut;
    head = (head + 1) % 32;
    freeBytes--;
  }
  uint8_t get() {                 // should only be called if buffer not empty
    freeBytes++;
    uint8_t oldtail = tail;
    tail = (tail + 1) % 32;
    return bytes[oldtail];    
  }
  uint8_t peek() {
    return bytes[tail];    
  }
  bool notEmpty() {
    return (freeBytes < 32);
  }
  bool notFull() {
    return (freeBytes > 0);
  }
};



/////////////////////////////////////////////////////////////
// Set Up Buffers and Define Buffer Maintenance Operations //
/////////////////////////////////////////////////////////////


// Buffer for raw HW MIDI input data.

_byte_buffer inputBuffer;


// Intermediate buffers for non-system-exclusive messages.
// We separate realtime and non-realtime messages. We don't have a buffer for
// system exclusive messages, because we ignore these and don't pass them though.

_byte_buffer realtimeBuffer;  
_message_buffer messageBuffer;   



// Buffer for raw HW MIDI output data.

_byte_buffer outputBuffer; 



// Take data from HW MIDI input, if available, and place it into the
// input buffer. This is safe to call if the input buffer is not full.
// This is time-sensitive. If you don't call this often enough, around
// once every 100μs, you risk missing data. This is why we needed to use
// non-blocking analog reads, since the ordinary analogRead() function 
// blocks for ~100μs while reading. 
 
void readInputMIDI() {
  if ( midiSerial.available() ) {
    inputBuffer.put(midiSerial.read());
  }
}



// Function to parse raw data from MIDI input buffer and place it
// in realtime and non-realtime message buffers. This is safe to call
// call only if both the realtime and message buffers are not full.
   
_message message;                    // For storing a message as we read it byte-by-byte.
uint8_t dataBytesRemaining = 0;      // How many data bytes are left in our message?

void parseByte() { 
  if (isRealtimeMessage(inputBuffer.peek())) {          // Handle realtime status bytes

    realtimeBuffer.put(inputBuffer.get());   // Put realtime messages in the realtime buffer
    
  } else if (isHeader(inputBuffer.peek())) {            // Handle non-realtime status bytes
     message.header = inputBuffer.get();                           // Fill out the information
     message.numDataBytes = numDataBytes(message.header);          // we can deduce about the 
     message.channelMessage = isChannelMessage(message.header);    // message from its header
     dataBytesRemaining = message.numDataBytes;  // We now know how many data bytes we expect                  
     if (dataBytesRemaining == 0) {
       messageBuffer.put(message);    // If we expect no data bytes, our message is complete.
     }

  } else if (dataBytesRemaining > 0) { // If we are expecting more data bytes, and see one...

    message.data[message.numDataBytes - dataBytesRemaining] = inputBuffer.get();  // Put the data
    dataBytesRemaining--;                                                         // into message
    if (dataBytesRemaining == 0) {                   // If we have received all the data bytes,
      messageBuffer.put(message);                    // put the message in the message buffer,
      dataBytesRemaining = message.numDataBytes;     // and prepare for next message with the
    }                                                // same header (known as "running status").
    

  } else {            // If we were not expecting a data byte but received one, either we dropped
                      // a byte, or are receiving system exclusive data (which we are not handling).
    inputBuffer.get();        // In either case, we should throw away the unexpected data.

  }
}


// Transfers messages from the message buffer to the output buffer, one byte at a
// time. Needs to be called repeatedly to ensure the entire message is transferred.
// This is safe to call at any time, and should be called often.

_message tempMessage;          // Stores a message while we transfer it a byte at a time.
uint8_t bytesTransferred;      // Keeps track of how many bytes of the message we have left.
bool readyForNextMessage = true;  

void transferMessageByte() {
  if (readyForNextMessage) {                    // If we're ready for the next message,
    if (messageBuffer.notEmpty()) {             // and a new message is available,
      tempMessage = messageBuffer.get();        // get it from the message buffer.
      readyForNextMessage = false;
      bytesTransferred = 0;
      /*
       * This would be the place to put processing of MIDI messages in. 
       * E.g., you could filter for Note On events and transpose them.
       * Simply apply whatever function to tempMessage you like.
       * Resetting readyForNextMessage to true here will filter that message.
       */
    }
  } else {
    if (outputBuffer.notFull()) {   // We only transfer a byte if output buffer has room.
      if (bytesTransferred == 0) {              
        outputBuffer.put(tempMessage.header);   // Transfer header if we haven't already.
        bytesTransferred++;
      } else if (bytesTransferred <= tempMessage.numDataBytes) {  // If message isn't over,
        outputBuffer.put(tempMessage.data[bytesTransferred-1]);   // transfer the next byte.
        bytesTransferred++;
      } else {                              // If message is over, don't do anything
        readyForNextMessage = true;         // Just prepare for next message
        bytesTransferred = 0;
      }
    }
  }
}



// Transfers messages from the realtime buffer to the output buffer
// Only needs to be called once to transfer a single message, since
// realtime messages have only a status byte and no data bytes.
// This is safe to call at any time, and should be called often.

void transferRealtime() {
  if (outputBuffer.notFull() && realtimeBuffer.notEmpty()) {
    outputBuffer.put(realtimeBuffer.get());
  }
}


// Takes a byte from the output buffer, if available, and sends it both 
// to the HW MIDI output  and to the Serial output. Safe to call at any
// time, and should be called as often as possible.

void sendByteOut() {
  if ( outputBuffer.notEmpty() ) {
    uint8_t byteToSend;
    byteToSend = outputBuffer.get();
    midiSerial.write(byteToSend);
    if (isHeader(byteToSend)) {
      Serial.println();
    }
    Serial.print(byteToSend, DEC);
    Serial.print(" ");
  }
}



// Performs all necessary buffer input, maintenance, output tasks.
// Interleaves various operations and repeats them to make sure that
// buffers do not fill up. This should be called from the main loop
// as often as possible. It should also be called every time a message 
// is added to the message buffer. If it is called every time a message
// is added to the message buffer, the buffers are guaranteed to never
// be full. 

void processBuffers() {
  sendByteOut();
  readInputMIDI();
  if (inputBuffer.notEmpty()) {
    parseByte(); 
  } 
  transferRealtime();
  transferMessageByte();
  transferMessageByte();
  readInputMIDI();
  if (inputBuffer.notEmpty()) {
    parseByte(); 
  }      
  transferMessageByte();
  transferMessageByte();
  readInputMIDI();
  if (inputBuffer.notEmpty()) {
    parseByte(); 
  }
  sendByteOut();
   transferMessageByte();
  transferMessageByte();
  sendByteOut();
  transferMessageByte();
  transferMessageByte();
  sendByteOut();
  transferMessageByte();
  transferMessageByte();
  readInputMIDI();
  if (inputBuffer.notEmpty()) {
    parseByte(); 
  }
  sendByteOut(); 
}


////////////////////////////
// Analog Input Variables //
////////////////////////////


// Indicates which analog pin A0 through A7 is being read.

char analogInputSelected = 0;                  


// Array to store values of analog pins.
// Should convert to a appropriate format
// if you want to use in a MIDI message

unsigned int analogVal[8] = {0,0,0,0,0,0,0,0};




///////////////////////////
// Initialize Interfaces //
///////////////////////////

void setup() {

  // Initialize analog interfaces 
  
  ADMUX = 0x40 + analogInputSelected;  // 0x40 = 0100000 (Default voltage; Right-adjust)
  ADCSRA = 0xAF;   // 0xAF = 10101111 (ADC On; Auto Trigger On; Interrupt Enabled; Clock Division = 128)
  ADCSRB = 0x40;   // 0x40 = 0100000 (Muxing On; Free running mode)
  bitWrite(ADCSRA, 6, 1);  // Start Conversion by Setting ADSC=1 in ADCSRA
  sei();                   // Enable Global Interrupts 

  /* Decreasing the last three bits of ADCSRA will increase the speed of the ADC, but
     will lose some accuracy. You might want to experiment to find the right balance. */

  
  // Initialize serial interfaces
  
  Serial.begin(115200);
  midiSerial.begin(31250); 
  Serial.println("Serial Communication Started");
} 



///////////////
// Main Loop //
///////////////

unsigned long int delaytracker = 0; // Used to space out analog MIDI messages

void loop() {

  readInputMIDI();    // These need to be called often to ensure MIDI messages 
  processBuffers();   // are not dropped and buffers do not fill up
  
  if ( millis() > delaytracker) { // If we've waited long enough...
    
    delaytracker = millis() + 2000; // schedule the next time we run this

    // Add MIDI messages to message buffer based on analog pin values
    
    messageBuffer.add(0xB0, 0x01, (analogVal[0] >> 3) % 128);
    processBuffers();
    messageBuffer.add(0xB0, 0x43, (analogVal[1] >> 3) % 128);
    processBuffers(); 
    messageBuffer.add(0xB0, 0x40, (analogVal[2] >> 3) % 128);
    processBuffers();
    
  }
  /*Put any code you like here! You can read the values in the array analogVal,
    use them to perform additional calculations. They will update continuously
    in the background, and code here will not be delayed by lengthy analog reads.
   /*Just make sure not to block for long periods of time, or you might miss
     incoming MIDI messages.*/
}



////////////////////////////////////////
//**********INTERRUPT ROUTINE*********//
//***ADC is in continuous read mode***//
//*Routine triggers when ADC is ready*//
//*Result stored in analogInput array*//
////////////////////////////////////////

ISR(ADC_vect) {
  analogVal[analogInputSelected] = ADCL;       // Store least significant bits
  analogVal[analogInputSelected] += ADCH << 8; // Store most significant bits

  analogInputSelected = (analogInputSelected + 1) % 8; // Select next input pin 
  ADMUX = 0x40 + analogInputSelected; // Tell ADC to read selected pin
}

/* If you only need a few pins, you can speed up the rate at which pins are read
   by using a different method to choose the next input pin here which skips the
   pins you don't need to read. */
