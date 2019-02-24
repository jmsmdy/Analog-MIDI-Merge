#include <AltSoftSerial.h>
AltSoftSerial midiSerial;


// Analog Input Variables
uint8_t analogInputSelected = 0;
unsigned int analogInputA;
unsigned int analogInputB;
unsigned int analogInputC;
bool analogInputReady = false;

// #define RXPIN 8
// #define TXPIN 9

// MIDI Input Variables
uint8_t statusByteBuffer; // stores status byte for running status
uint8_t incomingByte; // stores incoming byte
uint8_t messageBytesReceived;
bool midiMessageComplete = false;

void setup() {
//  TIMSK0 = 0x00;           // disable timer (causes anoying interrupts)
  ADMUX = 0x40;            // measuring on ADC0, right adjust, default 5v
  ADCSRA = 0xAF;           // AD-converter on, interrupt enabled, prescaler = 128
  ADCSRB = 0x40;           // AD channels MUX on, free running mode
  bitWrite(ADCSRA, 6, 1);  // Start the conversion by setting bit 6 (=ADSC) in ADCSRA
  sei();              // Set global interrupt flag
  Serial.begin(115200);
  midiSerial.begin(31250);
  Serial.println("Start");
}

// An array giving the number of data bytes for various channel
// messages, based on first four bits of the status byte
const uint8_t numDataBytesChannelStatus[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 1, 2, 3};
// 0 corresponds to data bytes (non-status bytes). 3 corresponds to
// system common or realtime messages, which must be handled separately,
// since their lengths may vary (the 3 is just a placeholder value)

// An array giving the number of data bytes for various system
// messages, base on last four bits of the status byte
const uint8_t numDataBytesSystemStatus[16] = { 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// 3 corresponds to system exclusive (this has a variable number of bytes),
// so wait for 11110111 byte to terminate system exclusive message. 
// 0100 and 0101 are undefined/reserved, so we allow them zero data bits,
// which may cause interference if they are ever added to the spec. 


// Circular Input Buffer for HW MIDI IN. Write to head, read from tail.
// Head is location of next byte to write, not last written byte.
#define inputBufferSize 32
uint8_t inputBuffer[inputBufferSize];
uint8_t inputBufferHead = 0;
uint8_t inputBufferTail = 0;
uint8_t inputBufferFreeBytes = inputBufferSize;

// Reads one byte from HW MIDI IN, places it in input buffer
// Needs to be called often to avoid missed bytes
void readInputMIDI() {
  if ( (inputBufferFreeBytes > 0) & midiSerial.available() ) {
    inputBuffer[inputBufferHead] = midiSerial.read();
    inputBufferHead = (inputBufferHead + 1) % inputBufferSize;
    inputBufferFreeBytes--;
    if ( inputBufferFreeBytes == 0 ) {
      Serial.println("Warning: HW MIDI Input Buffer Full"); 
    }
  }
}

/* // Gets a single byte from tail of input buffer, advances tail
// to free up room in buffer, and update # of free bytes.
// Should only be called if buffer is not empty.
uint8_t inputBufferGet() {
  inputBufferFreeBytes++;
  uint8_t temp = inputBuffer[inputBufferTail];
  inputBufferTail = (inputBufferTail + 1) % inputBufferSize;
  return temp; 
}*/

// Circular Output Buffer for HW MIDI OUT. Write to head, read from tail.
// Head is location of next byte to write, not last written byte.
#define outputBufferSize 32
uint8_t outputBuffer[outputBufferSize];
uint8_t outputBufferHead = 0;
uint8_t outputBufferTail = 0;
uint8_t outputBufferFreeBytes = outputBufferSize;
bool outputBufferMergeBlocked = false; // New MIDI messages can only be
                                       // merged if the last message is
                                       // is finished                                       
uint8_t runningStatusByte; // Stores running status byte, which is sent to
                           // re-establish running status if it is broken
                           // during the merging of new MIDI messages.
                           // Set to 0 if there is no running status.
bool runningStatusOn = false; // tracks whether running status is enabled

// Gets a single byte from tail of output buffer, advances tail
// to free up room in buffer, and update # of free bytes.
// Should only be called if buffer is not empty.
uint8_t outputBufferGet() {
  outputBufferFreeBytes++;
  uint8_t temp = outputBuffer[outputBufferTail];
  outputBufferTail = (outputBufferTail + 1) % outputBufferSize;
  return temp; 
}


// Circular Merge Buffer. Write to head, read from tail.
// Head is location of next byte to write, not last written byte.
#define mergeBufferSize 27
uint8_t mergeBuffer[mergeBufferSize]; // Stores MIDI messages to be merged.
uint8_t mergeBufferHead = 0;          // Messages must start with status byte
uint8_t mergeBufferTail = 0;          // and running status is not allowed.
uint8_t mergeBufferFreeBytes = mergeBufferSize; // keeps track of how many bytes
                                                // are free in merge buffer                                   

// Puts message in merge buffer, if there is room.
// Does nothing and returns false if there is not.
bool mergeBufferPut( uint8_t statusByte, uint8_t dataByte1, uint8_t dataByte2 ) {
  if ( mergeBufferFreeBytes >= 3 ) {
    mergeBuffer[mergeBufferHead] = statusByte;
    mergeBufferHead = (mergeBufferHead + 1) % mergeBufferSize;
    mergeBuffer[mergeBufferHead] = dataByte1;
    mergeBufferHead = (mergeBufferHead + 1) % mergeBufferSize;
    mergeBuffer[mergeBufferHead] = dataByte2;
    mergeBufferHead = (mergeBufferHead + 1) % mergeBufferSize;
    mergeBufferFreeBytes = mergeBufferFreeBytes - 3; 
    /* if ( mergeBufferHead > mergeBufferTail ) {
      mergeBufferFreeBytes = mergeBufferSize - (mergeBufferHead - mergeBufferTail);  
    } else {
      mergeBufferFreeBytes = mergeBufferTail - mergeBufferHead;
    }*/
    return true;
  } else {
    Serial.println("Warning: Merge Buffer Full");
    return false; 
  }
}

// Gets a new MIDI message from the merge buffer and merges it to the output buffer,
// if there is one available, and the output buffer has room. Returns false if either 
// the merge buffer is empty or output buffer does not have enough room.
bool mergeToOutputBuffer() {
  uint8_t messageFinalByteOffset = 0;
  if ( outputBufferMergeBlocked || (mergeBufferFreeBytes == mergeBufferSize) ) {
    return false; 
  } else if ( (mergeBuffer[mergeBufferTail] >> 4) == 0x0F ) { // If it's a system status byte...
    if ( (mergeBuffer[mergeBufferTail] & 0x0F) == 0 ) { // If it's a system exclusive byte...
      while ( mergeBuffer[mergeBufferHead + messageFinalByteOffset] != 0xF7 ) {
        messageFinalByteOffset++;    // Search for terminating byte (0xF7) of system
      }                              // exclusive message to find offset of final bit
    } else {
      messageFinalByteOffset = numDataBytesSystemStatus[mergeBuffer[mergeBufferTail] & 0x0F];
    }
  } else {
    messageFinalByteOffset = numDataBytesChannelStatus[mergeBuffer[mergeBufferTail] >> 4];    
  }
  if ( outputBufferFreeBytes > messageFinalByteOffset + 1 ) { // if enough room in output buffer
    outputBuffer[outputBufferHead] = mergeBuffer[mergeBufferTail];  // put status byte
    outputBufferHead = (outputBufferHead + 1) % outputBufferSize;   // in output buffer
    outputBufferFreeBytes--;
    mergeBufferTail = (mergeBufferTail + 1) % mergeBufferSize;
    mergeBufferFreeBytes++;
    while( messageFinalByteOffset != 0 ){                            // put remaining 
      outputBuffer[outputBufferHead] = mergeBuffer[mergeBufferTail]; // message bytes
      outputBufferHead = (outputBufferHead + 1) % outputBufferSize;  // in output buffer
      outputBufferFreeBytes--;
      mergeBufferTail = (mergeBufferTail + 1) % mergeBufferSize;
      mergeBufferFreeBytes++;
      messageFinalByteOffset--;
    }
    /*if ( outputBufferHead > outputBufferTail ) { // update # of free bytes in output buffer
      outputBufferFreeBytes = outputBufferSize - (outputBufferHead - outputBufferTail);  
    } else {
      outputBufferFreeBytes = outputBufferTail - outputBufferHead;
    }
    if ( mergeBufferHead > mergeBufferTail ) { // update # of free bytes in merge buffer
      mergeBufferFreeBytes = mergeBufferSize - (mergeBufferHead - mergeBufferTail);  
    } else {
      mergeBufferFreeBytes = mergeBufferTail - mergeBufferHead;
    }*/
    if ( runningStatusOn ) { // restablish running status, if was broken by merge
      outputBuffer[outputBufferHead] = runningStatusByte; 
      outputBufferHead = (outputBufferHead + 1) % outputBufferSize;
      outputBufferFreeBytes--;
    }
    return true;
  } else {
    return false;
  }
}

void transferByte() {
  outputBuffer[outputBufferHead] = inputBuffer[inputBufferTail];
  outputBufferHead = (outputBufferHead + 1) % outputBufferSize;
  outputBufferFreeBytes--;
  inputBufferTail = (inputBufferTail + 1) % inputBufferSize;
  inputBufferFreeBytes++;
}

void purgeByte() {
  inputBufferTail = (inputBufferTail + 1) % inputBufferSize;
  inputBufferFreeBytes++;
}

// Gets a byte from the input buffer and puts it in the output buffer,
// if there is one available, and the output buffer has room. Returns false if either 
// the input buffer is empty or output buffer does not have enough room.
uint8_t mergeBlock = 0; // Tracks expected bytes until merge block can be lifted
bool systemExclusiveMode = false;
bool inputToOutputBuffer() {
  if ( (inputBufferFreeBytes == inputBufferSize) || (outputBufferFreeBytes == 0) ) {
      return false; 
  } else if ( (inputBuffer[inputBufferTail] >> 3) == 0x1F ) { // if a realtime status byte
      transferByte(); // transfer it from input to output buffer 
      return true;    // don't do anything else (realtime messages do not affect running status)
  } else if ( mergeBlock > 0 ) { // if expecting data byte and not in system exclusive mode
      if ( (inputBuffer[inputBufferTail] >> 7) == 0 ) { // if indeed we find a data byte
        transferByte();
        mergeBlock--; // we are now expecting one fewer data byte 
        if ( mergeBlock == 0 ) { // if message is complete
          outputBufferMergeBlocked = false; // lift the block on merging new messages
        }
        return true;
      } else { // if we receive some other status byte (i.e. if something went wrong)
        mergeBlock = 0;                    // reset # of data bits expected to zero
        outputBufferMergeBlocked = false;  // remove block on merging new messages
        runningStatusOn = false;           // set running status to off
        runningStatusByte = 0;
        Serial.println("Error: expected data byte, received status byte");
        return false;
      }    
  } else if ( systemExclusiveMode ) {
      if ( (inputBuffer[inputBufferTail] >> 7) == 0 ) { // if see a data byte
        transferByte();
        return true; 
      } else if ( inputBuffer[inputBufferTail] == 0xF7 ) { // if see sys. excl. termination byte
        transferByte();
        systemExclusiveMode = false;
        outputBufferMergeBlocked = false;
        return true;
      } else { // if see unexpected status byte (i.e. something went wrong)
        systemExclusiveMode = false; // reset a bunch of things
        runningStatusByte = 0;
        runningStatusOn = false;
        outputBufferMergeBlocked = false;
        Serial.println("Error: expected sys. excl. data byte, received status byte");
        return false;
      }
  } else if ( inputBuffer[inputBufferTail] == 0xF0 ) { // if see sys. exclusive start byte
      systemExclusiveMode = true;
      runningStatusOn = false; // no running status allowed in system exclusive mode
      runningStatusByte = 0;
      outputBufferMergeBlocked = true; // no merging until system exclusive message is over
      transferByte();
      return true;
  } else if ( runningStatusOn ) { 
      if ( (inputBuffer[inputBufferTail] >> 7) == 0 ) {// if see data byte
        outputBufferMergeBlocked = true;
        mergeBlock = numDataBytesChannelStatus[runningStatusByte >> 4]; 
        return true; // will transfer byte on next call
      } else {
        runningStatusOn = false;
        runningStatusByte = 0;
        outputBufferMergeBlocked = false;
        return true; // running status turned off, will re-evaluate on next call
      }
  } else if (inputBuffer[inputBufferTail] < 0xF0 ) { // if non-system byte
      if ( (inputBuffer[inputBufferTail] >> 7) == 1 ) { // if non-system status byte
        runningStatusByte = inputBuffer[inputBufferTail];
        mergeBlock = numDataBytesChannelStatus[runningStatusByte >> 4]; 
        if ( mergeBlock > 0 ) {
          runningStatusOn = true;
          outputBufferMergeBlocked = true; 
        } else {
          outputBufferMergeBlocked = false;
        }
        transferByte();
        return true;
      } else { // otherwise we received an unexpected data byte (i.e. something went wrong)
        purgeByte(); // throw away unexpected byte
        runningStatusOn = false;
        runningStatusByte = 0;
        outputBufferMergeBlocked = false;
        Serial.println("Error: expected status byte, received data byte.");
        return false;
      }
 } else { // only possibility remaining is we received a non-realtime system status byte
      runningStatusByte = 0;
      runningStatusOn = false;
      mergeBlock = numDataBytesSystemStatus[inputBuffer[inputBufferTail] & 0x0F];
      if ( mergeBlock > 0 ) {
        outputBufferMergeBlocked = true; 
      } else {
        outputBufferMergeBlocked = false;
      }
      transferByte();
      return true; // will merge byte next time this is run
 }
}      

uint8_t outputByte;
void loop()
{
  unsigned long i;
  for (i = 0; i < 20000; i++) {
    readInputMIDI();
    mergeToOutputBuffer();
    readInputMIDI();
    inputToOutputBuffer();
    readInputMIDI();
    if ( outputBufferFreeBytes < outputBufferSize ) {
      outputByte = outputBufferGet();
      midiSerial.write(outputByte);
      if ( (outputByte >> 7) == 1 ) {Serial.println("");} 
      Serial.print(outputByte , DEC);
      Serial.print(" ");
    }
  }
  //Serial.println("reading analogue input");
  mergeBufferPut(0xB0, 0x01, (analogInputA >> 3) % 128); // modulation on channel 1
  for (i = 0; i < 20000; i++) {
    readInputMIDI();
    mergeToOutputBuffer();
    readInputMIDI();
    inputToOutputBuffer();
    readInputMIDI();
    if ( outputBufferFreeBytes < outputBufferSize ) {
      outputByte = outputBufferGet();
      midiSerial.write(outputByte);
      if ( (outputByte >> 7) == 1 ) {Serial.println("");} 
      Serial.print(outputByte , DEC);
      Serial.print(" ");
    }
  }
  //Serial.println("reading analogue input");
  mergeBufferPut(0xB0, 0x43, (analogInputB >> 3) % 128); // soft pedal on channel 1
  for (i = 0; i < 20000; i++) {
    readInputMIDI();
    mergeToOutputBuffer();
    readInputMIDI();
    inputToOutputBuffer();
    readInputMIDI();
    if ( outputBufferFreeBytes < outputBufferSize ) {
      outputByte = outputBufferGet();
      midiSerial.write(outputByte);
      if ( (outputByte >> 7) == 1 ) {Serial.println("");} 
      Serial.print(outputByte , DEC);
      Serial.print(" ");
    }
  }
  //Serial.println("reading analogue input");
  mergeBufferPut(0xB0, 0x40, (analogInputC >> 3) % 128); // damper pedal on channel 1

}

////////////////////////////////////////
//**********Interrupt Routine*********//
//***ADC is in continuous read mode***//
//*Routine triggers when ADC is ready*//
//*Result stored in analogInputX vars*//
////////////////////////////////////////
ISR(ADC_vect) {
  switch (analogInputSelected) {
    case 0:                      // If pin 0 is selected:
      analogInputA = ADCL;       //   store lower byte ADC
      analogInputA += ADCH << 8; //   store higher bytes ADC
      analogInputSelected = 1;   //   select pin 1 
      ADMUX = 0x41;              //   tell ADC to read pin 1
      break;
    case 1:                      // If pin 1 is selected:
      analogInputB = ADCL;       //   store lower byte ADC
      analogInputB += ADCH << 8; //   store higher bytes ADC
      analogInputSelected = 2;   //   select pin 2
      ADMUX = 0x42;              //   tell ADC to read pin 2
      break;
    case 2:                      // If pin 2 is selected:
      analogInputC = ADCL;       //   store lower byte ADC
      analogInputC += ADCH << 8; //   store higher bytes ADC
      analogInputSelected = 0;   //   select pin 0
      ADMUX = 0x40;              //   tell ADC to read pin 0
      break;
  }
  analogInputReady = 1;
}
