/* ************************************************************************** */

#ifndef _MESSAGE_H    /* Guard against multiple inclusion */
#define _MESSAGE_H

/* ************************************************************************** */
/* Message Definitions                                                        */
/* ************************************************************************** */

// *** FromTo ***
// Raspberry Pi Address
#define MSG_ADDR_PI ((unsigned char)0x00)

// Rover 1 module addresses
#define MSG_ADDR_R1_COMM_TH ((unsigned char)0x01)
#define MSG_ADDR_R1_SENS_TH ((unsigned char)0x02)
#define MSG_ADDR_R1_NAV_TH ((unsigned char)0x03)
#define MSG_ADDR_R1_MOT_TH ((unsigned char)0x04)
#define MSG_ADDR_R1_TIME_ISR ((unsigned char)0x05)
#define MSG_ADDR_R1_ENC_ISR ((unsigned char)0x06)

// Rover 2 module addresses
#define MSG_ADDR_R2_COMM_TH ((unsigned char)0x09)
#define MSG_ADDR_R2_SENS_TH ((unsigned char)0x0A)
#define MSG_ADDR_R2_NAV_TH ((unsigned char)0x0B)
#define MSG_ADDR_R2_MOT_TH ((unsigned char)0x0C)
#define MSG_ADDR_R2_TIME_ISR ((unsigned char)0x0D)
#define MSG_ADDR_R2_ENC_ISR ((unsigned char)0x0E)

// *** Subject ***
// Subjects
#define MSG_SUBJECT_ADC_DATA ((unsigned char)0x00)
#define MSG_SUBJECT_LINE_FOLLOWER_DATA ((unsigned char)0x01)
#define MSG_SUBJECT_SIGNAL ((unsigned char)0x02)

// *** Signal ***
// Sensor thread to nav/mapping thread signals
#define MSG_SIG_LEFT_BLOCKED ((unsigned char)0xC0)
#define MSG_SIG_RIGHT_BLOCKED ((unsigned char)0x03)
#define MSG_SIG_FORWARD_BLOCKED ((unsigned char)0x18)

// Nav/mapping thread to motor thread signals
#define MSG_SIG_GO_STRAIGHT ((unsigned char)0x03)
#define MSG_SIG_TURN_RIGHT ((unsigned char)0x04)
#define MSG_SIG_TURN_LEFT ((unsigned char)0x05)

// Inter-rover signals
#define MSG_SIG_START_MAPPING ((unsigned char)0x06)
#define MSG_SIG_START_L_TURN ((unsigned char)0x07)
#define MSG_SIG_START_R_TURN ((unsigned char)0x08)
#define MSG_SIG_COMPLETED_TURN ((unsigned char)0x09)
#define MSG_SIG_IN_POSITION ((unsigned char)0x0A)
#define MSG_SIG_LOST ((unsigned char)0x0B)

/* ************************************************************************** */
/* Message Struct Definitions                                                 */
/* ************************************************************************** */

// Use to send start/stop/acks/commands/etc.
// Size: 9 bytes
typedef struct signalMsg {
  unsigned char subject;
  unsigned char fromTo;
  unsigned long long checksum;
  unsigned char signal;
}SIGNAL_MSG;

// Use to send two ADC sensor datas
// Size: 16 bytes
typedef struct ADCMsg {
  unsigned long long subject; //these need to be this size to be able to shift them 40 bits
  unsigned long long fromTo;
  unsigned long long checksum;
  unsigned short data1;
  unsigned short data2;
}ADC_MSG;

// Use to send three line following sensor datas
// Size: 9 bytes
typedef struct LFMsg {
  unsigned long long subject;
  unsigned long long fromTo;
  unsigned long long checksum;
  unsigned int frontSensor;
  unsigned int leftSensor;
  unsigned int rightSensor;
}LF_MSG;

/* ************************************************************************** */
/* Serialization/Deserialization Function Definitions                         */
/* ************************************************************************** */

SIGNAL_MSG createSignalMsg(unsigned char subject, unsigned char from, unsigned char to, unsigned char signal);
void packSignalMsg(unsigned char *buffer, SIGNAL_MSG message);
SIGNAL_MSG unpackSignalMsg(unsigned char *buffer, int *status);

ADC_MSG createADCMsg(unsigned char subject, unsigned char from, unsigned char to, unsigned short data1, unsigned short data2);
void packADCMsg(unsigned char *buffer, ADC_MSG message);
ADC_MSG unpackADCMsg(unsigned long long *buffer, int *status);

LF_MSG createLFMsg(unsigned char subject, unsigned char from, unsigned char to, unsigned char front, unsigned char left, unsigned char right);
void packLFMsg(unsigned char *buffer, LF_MSG message);
LF_MSG unpackLFMsg(unsigned long long *buffer, int *status);

#endif

/* *****************************************************************************
 End of File
 */
