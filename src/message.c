/* ************************************************************************** */

#include "message.h"

/* SIGNAL MESSAGE */

SIGNAL_MSG createSignalMsg(unsigned char subject, unsigned char from, unsigned char to, unsigned char signal){
  SIGNAL_MSG created;
  unsigned int poly, value;
  created.subject = subject;
  created.fromTo = (from << 4) | to;
  created.signal = signal;

  value = (created.subject << 16)|(created.fromTo  << 8)|created.signal; //24 bits
  value = value << 9; //shifting to 33 bits for CRC
  poly = 0x04C11DB7; //polynomial for calculating CRC32
  
  created.checksum = value % poly; //new CRC32 checksum

  return created;
}

void packSignalMsg(unsigned char *buffer, SIGNAL_MSG message){
  buffer[0] = 0x02; //start-of-text
  buffer[1] = message.subject;
  buffer[2] = message.fromTo;

  unsigned long long shiftBuffer = message.checksum;
  buffer[3] = (unsigned char)(shiftBuffer & 0xFF);
  buffer[4] = (unsigned char)((shiftBuffer >> 8) & 0xFF);
  buffer[5] = (unsigned char)((shiftBuffer >> 16) & 0xFF);
  buffer[6] = (unsigned char)((shiftBuffer >> 24) & 0xFF);  
  buffer[7] = message.signal;
  buffer[8] = 0x00;
  buffer[9] = 0x00;
  buffer[10] = 0x00;
  buffer[11] = 0x00;
  buffer[12] = 0x00;
  buffer[13] = 0x00;
  buffer[14] = 0x00;
  buffer[15] = 0x03; //end-of-text
  
}

SIGNAL_MSG unpackSignalMsg(unsigned char *buffer, int *status){
  SIGNAL_MSG toReturn;
  toReturn.subject = buffer[1];
  toReturn.fromTo = buffer[2];
  toReturn.checksum = (buffer[6]<<24)|(buffer[5]<<16)|(buffer[4] << 8) | buffer[3];
  toReturn.signal = buffer[7];
  int value, poly, checker;
  
  value = (toReturn.subject << 16)|(toReturn.fromTo  << 8)|toReturn.signal; //24 bits
  value = value << 9; //shifting to 33 bits for CRC
  poly = 0x04C11DB7; //polynomial for calculating CRC32

  checker = value % poly;
  
  if (checker == toReturn.checksum){
    *status = 0;
  } else {
    *status = -1;
  }

  return toReturn;
}

/* ADC MESSAGE */

ADC_MSG createADCMsg(unsigned char subject, unsigned char from, unsigned char to, unsigned short data1, unsigned short data2){
  ADC_MSG created;
  unsigned long long poly, value; //using long longs bc this is 8 bytes
  created.subject = subject;
  created.fromTo = (from << 4) | to;
  created.data1 = data1;//these are two bytes each
  created.data2 = data2;//2 bytes
  
  value = (created.subject << 40)|(created.fromTo  << 32)|(created.data1<<16)|created.data2; //48 bits
  value = value << 13; //shifting to 65 bits for CRC
  poly = 0x42F0E1EBA9EA3693; //polynomial for calculating CRC64
  
  created.checksum = value % poly; //new CRC64 checksum
  
  //created.checksum = created.subject ^ created.fromTo;
  //created.checksum = created.checksum ^ created.data1;
  //created.checksum = created.checksum ^ created.data2;

  return created;
}

void packADCMsg(unsigned char *buffer, ADC_MSG message){
  buffer[0] = 0x02; //start-of-text
  buffer[1] = message.subject;
  buffer[2] = message.fromTo;

  unsigned long long shiftBuffer = message.checksum;
  buffer[3] = (unsigned char)(shiftBuffer & 0xFF);
  buffer[4] = (unsigned char)((shiftBuffer >> 8) & 0xFF);
  buffer[5] = (unsigned char)((shiftBuffer >> 16) & 0xFF);
  buffer[6] = (unsigned char)((shiftBuffer >> 24) & 0xFF);
  buffer[7] = (unsigned char)((shiftBuffer >> 32) & 0xFF);
  buffer[8] = (unsigned char)((shiftBuffer >> 40) & 0xFF);
  buffer[9] = (unsigned char)((shiftBuffer >> 48) & 0xFF);
  buffer[10] = (unsigned char)((shiftBuffer >> 56) & 0xFF);
  
  shiftBuffer = message.data1;
  buffer[11] = (unsigned char)(shiftBuffer & 0xFF);
  buffer[12] = (unsigned char)((shiftBuffer >> 8) & 0xFF);

  shiftBuffer = message.data2;
  buffer[13] = (unsigned char)(shiftBuffer & 0xFF);
  buffer[14] = (unsigned char)((shiftBuffer >> 8) & 0xFF);

  buffer[15] = 0x03; //end-of-text
}

ADC_MSG unpackADCMsg(unsigned long long *buffer, int *status){ //I had to make the buffer 64 bits, 
  ADC_MSG toReturn;                                            // making it less causes warning
  toReturn.subject = buffer[1];
  toReturn.fromTo = buffer[2];
  toReturn.checksum = (buffer[10] << 56) |(buffer[9] << 48) |(buffer[8] << 40) |(buffer[7] << 32) |(buffer[6] << 24) |
          (buffer[5] << 16) |(buffer[4] << 8) | buffer[3];
  toReturn.data1 = (buffer[6] << 8) | buffer[5];
  toReturn.data2 = (buffer[8] << 8) | buffer[7];
  
  unsigned long long value, poly, checker;
  value = (toReturn.subject << 40)|(toReturn.fromTo  << 32)|(toReturn.data1<<16)|toReturn.data2; //48 bits
  value = value << 13;       //shifting to 65 bits for CRC
  poly = 0x42F0E1EBA9EA3693; //polynomial for calculating CRC64
  
  checker = value % poly;

  if(checker == toReturn.checksum){
    *status = 0;
  } else {
    *status = -1;
  }

  return toReturn;
}

/* LINE FOLLOWING MESSAGE */

LF_MSG createLFMsg(unsigned char subject, unsigned char from, unsigned char to, unsigned char front, unsigned char left, unsigned char right){
  LF_MSG created;
  created.subject = subject;
  created.fromTo = (from << 4) | to;
  created.frontSensor = front;
  created.leftSensor = left;
  created.rightSensor = right;
  
  unsigned long long poly, value;
  
  value = (created.subject << 32)|(created.fromTo  << 24)|(created.frontSensor<<16)|
          (created.leftSensor<<8) | created.rightSensor; //40 bits
  value = value << 25; //shifting to 65 bits for CRC
  poly = 0x42F0E1EBA9EA3693; //polynomial for calculating CRC64
  
  created.checksum = value % poly; //new CRC64 checksum

  return created;
}

void packLFMsg(unsigned char *buffer, LF_MSG message){
  buffer[0] = 0x02; //start-of-text
  buffer[1] = message.subject;
  buffer[2] = message.fromTo;

  unsigned long long shiftBuffer = message.checksum;
  buffer[3] = (unsigned char)(shiftBuffer & 0xFF);
  buffer[4] = (unsigned char)((shiftBuffer >> 8) & 0xFF);
  buffer[5] = (unsigned char)((shiftBuffer >> 16) & 0xFF);
  buffer[6] = (unsigned char)((shiftBuffer >> 24) & 0xFF);
  buffer[7] = (unsigned char)((shiftBuffer >> 32) & 0xFF);
  buffer[8] = (unsigned char)((shiftBuffer >> 40) & 0xFF);
  buffer[9] = (unsigned char)((shiftBuffer >> 48) & 0xFF);
  buffer[10] = (unsigned char)((shiftBuffer >> 56) & 0xFF);

  buffer[11] = message.frontSensor;
  buffer[12] = message.leftSensor;
  buffer[13] = message.rightSensor;
  buffer[14] = 0x00;
  buffer[15] = 0x03;
}

LF_MSG unpackLFMsg(unsigned long long *buffer, int *status){
  LF_MSG toReturn;
  toReturn.subject = buffer[1];
  toReturn.fromTo = buffer[2];
  toReturn.checksum = (buffer[10] << 56) |(buffer[9] << 48) |(buffer[8] << 40) |(buffer[7] << 32) |(buffer[6] << 24) |
          (buffer[5] << 16) |(buffer[4] << 8) | buffer[3];
  toReturn.frontSensor = buffer[11];
  toReturn.leftSensor = buffer[12];
  toReturn.rightSensor = buffer[13];

 unsigned long long value, poly, checker;
  value = (toReturn.subject << 32)|(toReturn.fromTo  << 24)|(toReturn.frontSensor<<16)|
          (toReturn.leftSensor<<8) | toReturn.rightSensor; //40 bits
  value = value << 13;       //shifting to 65 bits for CRC
  poly = 0x42F0E1EBA9EA3693; //polynomial for calculating CRC64
  
  checker = value % poly;


  if(checker == toReturn.checksum){
    *status = 0;
  } else {
    *status = -1;
  }

  return toReturn;
}

/* *****************************************************************************
 End of File
 */
