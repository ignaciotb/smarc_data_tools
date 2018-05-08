/* ---------------------------------------------------------------------- */ 
/* SonarMessages.h                                                        */ 
/* ---------------------------------------------------------------------- */ 
/*                                                                        */ 
/* Describes the standard sonar messages which can be sent between a      */ 
/* topside and the Surface Interface Unit.  Messages always consist of    */ 
/* 2 parts:                                                               */ 
/*      1) A header which contains the message type and length            */ 
/*      2) The actual data                                                */ 
/*                                                                        */ 
/* Note that two communications circuits (sockets) are used.  Socket 1600 */ 
/* is used for commands and socket 1601 is used for data.  Commands are   */ 
/* sent from a topside, and the bottom unit may return status on the      */ 
/* command socket.  Data is only one way - from the bottom unit to the    */ 
/* topside and returns a header followed by SegyDataType data only.       */ 
/*                                                                        */ 
/* The subbottom unit shall have predefined defaults for all parameters,  */ 
/* therefore it is only necessary to send the messages that change the    */ 
/* defaults.  Defaults are stored in a file and can be changed using a    */ 
/* text editor.  A minimum topside interface can receive data without     */ 
/* sending a single command.                                              */ 
/*                                                                        */ 
/* ---------------------------------------------------------------------- */ 
/*                                                                        */ 
/* (c) Copyright 1998, 1999  EdgeTech,                                    */ 
/*                                                                        */ 
/* This file contains proprietary information, and trade secrets of       */ 
/* EdgeTech, and may not be disclosed or reproduced without the prior     */ 
/* written consent of EdgeTech.                                           */ 
/*                                                                        */ 
/* EdgeTech is not responsible for the consequences of the use or misuse  */ 
/* of this software, even if they result from defects in it.              */ 
/*                                                                        */ 
/* ---------------------------------------------------------------------- */ 

#ifndef __SonarMessages_H__
#define __SonarMessages_H__


/* ---------------------------------------------------------------------- */ 
/* Includes                                                               */ 
/* ---------------------------------------------------------------------- */ 

#include "time.h"
#include "PublicMessageOffsets.h"


/* ---------------------------------------------------------------------- */ 
/* Structure defines                                                      */ 
/* ---------------------------------------------------------------------- */ 

#define FILE_NAME_SIZE                     (80)
#define SONAR_MESSAGE_STRING_LENGTH       (256)


/* ---------------------------------------------------------------------- */ 
/*  Message Header                                                        */ 
/* ---------------------------------------------------------------------- */ 
/* All messages are preceeded with a header which indicates the size of   */ 
/* the message to follow in bytes and its type.  The header is of type    */ 
/* SonarMessageHeaderType. A version number is included to accomodate     */ 
/* future changes / extensions in the protocol.                           */ 
/* ---------------------------------------------------------------------- */ 

typedef struct
{
  /* Marker for the start of header                                       */ 
  unsigned short int startOfMessage;

  /* The version of the protocol in use                                   */ 
  unsigned char version;

  /* Session Identifier                                                   */ 
  unsigned char sessionID;

  /* The message format as per SonarMessageType                           */ 
  unsigned short int sonarMessage;

  /* The action to perform as per SonarCommandType                        */ 
  unsigned char sonarCommand;

  /* Indicates subsystem (0 is first) for a multi-system device.          */ 
  unsigned char subSystem;

  /* Indicates channel (0 is first) for a multi-channel subsystem.        */ 
  unsigned char channel;

  /* Sequence number of message.  A reply to this message will contain    */ 
  /* this value.  A topside can optionally use this field to track replys */ 
  unsigned char sequence;

  /* Header space reserved for future use.                                */ 
  unsigned char reservedHeader[2];

  /* Size of message in bytes to follow                                   */ 
  long int byteCount;         

} SonarMessageHeaderType;


/* ---------------------------------------------------------------------- */ 
/* Header defines                                                         */ 
/* ---------------------------------------------------------------------- */ 

/* Start of header word                                                   */ 
#define SONAR_MESSAGE_HEADER_START         (0x1601)

/* The version of the protocol in use                                     */ 
#define SONAR_PROTOCOL_CURRENT_VERSION       (0x03)


/* ---------------------------------------------------------------------- */ 
/* Command Types                                                          */ 
/* ---------------------------------------------------------------------- */ 
/* Command type(sonarCommand in header).  A topside will send SET         */ 
/* commands, which return no values, GET commands, which return the       */ 
/* current parameter set.  The bottom unit only sends REPLY messages in   */ 
/* response to a GET or unsolicited data.                                 */ 
/* ---------------------------------------------------------------------- */ 

typedef enum
{
  /* Send a value to black box or execute a command                       */ 
  SONAR_COMMAND_SET = 0,     

  /* Request current value / last value of item. For a GET, the byteCount */ 
  /* should be 0 (any data sent is ignored)                               */ 
  SONAR_COMMAND_GET,

  /* Reply to a COMMAND_GET or unsolicited error msgs                     */ 
  SONAR_COMMAND_REPLY,   

  /* Error replay to a command.  The command was not executed.            */ 
  /* In this case, the return value is the error (SonarMessageLongType)   */ 
  /* as defined by SonarMessageErrorType.                                 */ 
  SONAR_COMMAND_ERROR
  
} SonarCommandType;


/* ---------------------------------------------------------------------- */ 
/* Error codes that can be returned as SONAR_COMMAND_ERROR                */ 
/* ---------------------------------------------------------------------- */ 

typedef enum
{
  /* 0: No error                                                          */ 
  SONAR_MESSAGE_ERROR_NONE,

  /* 1: General command failure indicator                                 */ 
  SONAR_MESSAGE_ERROR_FAILED,

  /* 2: Message size does not match expected size                         */ 
  SONAR_MESSAGE_ERROR_DATA_SIZE,

  /* 3: Subsystem number is not present in this system                    */ 
  SONAR_MESSAGE_ERROR_SUBSYSTEM,

  /* 4: Channel number is not present in this system                      */ 
  SONAR_MESSAGE_ERROR_CHANNEL,

  /* 5: sonarMessage field contains an unknown command                    */ 
  SONAR_MESSAGE_ERROR_UNKNOWN,

  /* 6: Pulse file error.  Pulse not loaded                               */ 
  SONAR_MESSAGE_ERROR_PULSE_FILE,

} SonarMessageErrorType;


/* ---------------------------------------------------------------------- */ 
/* Message Types                                                          */ 
/* ---------------------------------------------------------------------- */ 
/* sonarMessage field indicates the type of data to follow.               */ 
/* ---------------------------------------------------------------------- */ 

typedef enum
{
  /* -------------------------------------------------------------------- */ 
  /* Overall commands.                                                    */ 
  /* For the following messages, the channel and subSystem should be 0.   */ 
  /* -------------------------------------------------------------------- */ 

  /* Null message - can be used to test communications                    */ 
  SONAR_MESSAGE_NONE = MESSAGE_OFFSET_STANDARD,

  /* Reset to default values (No Data) on set                             */ 
  SONAR_MESSAGE_SYSTEM_RESET,   

  /* Get or set the time (TimestampType)                                  */ 
  /* Note that because of the nagle algorithm on sockets, the actual      */ 
  /* message can be delayed.  To set the time with greater accuracy, say, */ 
  /* within 10 ms, the nagle algorithm should be disabled by the sender.  */ 
  /* Or a subsequent message bigger than the maximum network packet size  */ 
  /* should be sent following this message (usually about 1600 bytes).    */ 
  /* NOTE: The SONAR_MESSAGE_NONE message can be any size desired.        */ 
  SONAR_MESSAGE_SYSTEM_TIME,    

  /* Change the time by the delta in milliseconds (SonarMessageLongType)  */ 
  /* NOTE that in order to get accurate time sync, a delta must be set via*/ 
  /* this message, and it must be based on the GetTickCount() function.   */ 
  /* The normal GetSystemTime() is only accurate to about 50 ms.          */ 
  SONAR_MESSAGE_TIME_DELTA,

  /* Get the software version * 1000.0 (SonarMessageLongType)             */ 
  SONAR_MESSAGE_SYSTEM_VERSION,

  /* Status request / returned from black box (SonarMessageStatusType)    */ 
  /* A set can be used to reset status values                             */ 
  SONAR_MESSAGE_SYSTEM_STATUS = MESSAGE_OFFSET_STATUS,
  
  /* Alive messages are echoed back on the same connection.  This allows  */ 
  /* for the detection of lost connection links.  Accepts a long parameter*/ 
  /* (SonarMessageLongType) which is echoed back to the caller and is     */ 
  /* typically a sequence number.                                         */ 
  SONAR_MESSAGE_ALIVE,


  /* -------------------------------------------------------------------- */ 
  /* Sonar Return Data                                                    */ 
  /* -------------------------------------------------------------------- */ 

  /* Subbottom data returned to topside (SegyDataType)                    */ 
  SONAR_MESSAGE_DATA = MESSAGE_OFFSET_DATA,

  /* Window for data transmission (SonarMessageWindowType)                */ 
  SONAR_MESSAGE_DATA_NETWORK_WINDOW, 

  /* Sidescan data returned to topside (SidescanHeaderType)               */ 
  SONAR_MESSAGE_DATA_SIDESCAN,

  /* Activate / Deactivate return data type.  The data for the subsystem  */ 
  /* / channel specified in the header is activated / deactivated         */ 
  /* (0 - deactivate : 1 - activate)   (SonarMessageLongType)             */ 
  SONAR_MESSAGE_DATA_ACTIVE,


  /* -------------------------------------------------------------------- */ 
  /* Processing of data                                                   */ 
  /* -------------------------------------------------------------------- */ 

  /* Window for processing optimization (SonarMessageWindowType)          */ 
  SONAR_MESSAGE_PROCESSING_ENHANCE_WINDOW = MESSAGE_OFFSET_PROCESS,

  /* Samples to ignore due to direct path on AGC, normalization           */ 
  /* algorithms (SonarMessageLongType)                                    */ 
  SONAR_MESSAGE_PROCESSING_DIRECT_PATH,


  /* -------------------------------------------------------------------- */ 
  /* Management of pulses                                                 */ 
  /* -------------------------------------------------------------------- */ 

  /* Enable/disable ping: 0 => disable, 1=>enable, 2 => single ping only  */ 
  /* (SonarMessageLongType)                                               */ 
  SONAR_MESSAGE_PING = MESSAGE_OFFSET_PULSES,

  /* 1000.0 * DAC gain to scale outgoing pulse by (SonarMessageLongType)  */ 
  SONAR_MESSAGE_PING_GAIN,

  /* Set to reset to top of list (no parameter), get to get the next ping */ 
  /* returns(SonarMessagePingType).                                       */ 
  /* On a set you can either pass no data or a single value               */ 
  /* (SonarMessageLongType) with the count of maximum number of pulses    */ 
  /* to return (Default of 1).  If the count is greater than 1 then an    */ 
  /* array of SonarMessagePingType values will be returned.  The end of   */ 
  /* all pulses is indicated by a zero pulse name record.  A maximum count*/ 
  /* of 30 pulses is returned at a time.                                  */ 
  SONAR_MESSAGE_PING_LIST,   

  /* Select an outgoing pulse, matched filter pair                        */ 
  /* (SonarMessageStringType)                                             */ 
  SONAR_MESSAGE_PING_SELECT,  

  /* Number of pings pers second required * 1000                          */ 
  /* (SonarMessageLongType)                                               */ 
  /* Actual ping rate may be slightly lower (2048 sample granuality       */ 
  SONAR_MESSAGE_PING_RATE,

  /* Set trigger for internal(0) or external(1) (SonarMessageLongType)    */ 
  SONAR_MESSAGE_PING_TRIGGER,

  /* Set delay for external trigger (SonarMessageLongType)                */ 
  SONAR_MESSAGE_PING_DELAY,

  /* Used to GET the maximum number of samples for a given sample rate.   */ 
  /* Send a SonarMessageLongType with 1000 * the ping rate in Hz.         */ 
  /* OR Send not data to used the last ping rate set.                     */ 
  /* Returns a SonarMessageLongType with the maximum number of samples    */ 
  /* that can be received before the ping rate must be slowed down.       */ 
  SONAR_MESSAGE_PING_MAX_SAMPLES,

  /* Used to set the ping rate for sidescan systems.  Sets the ping rate  */ 
  /* based on the range in meters. (SonarMessageLongType)                 */ 
  SONAR_MESSAGE_PING_RANGE,


  /* -------------------------------------------------------------------- */ 
  /* ADC Control                                                          */ 
  /* -------------------------------------------------------------------- */ 

  /* Set gain factor for ADC when AGC disabled (SonarMessageLongType)     */ 
  /* Value is * 1000.0 (only 1,2,4,8 supported)                           */ 
  SONAR_MESSAGE_ADC_GAIN = MESSAGE_OFFSET_ADC,

  /* 0=> disable, 1=> enable (Automatic Gain Control)                     */ 
  /* (SonarMessageLongType)                                               */ 
  SONAR_MESSAGE_ADC_AGC,

  /* ADC rate in Hz * 1000.  This is a read only value and changes with   */ 
  /* the pulse selected (SonarMessageLongType)                            */ 
  SONAR_MESSAGE_ADC_RATE,

} SonarMessageType;


/* ---------------------------------------------------------------------- */ 
/* Message Data Structures                                                */ 
/* ---------------------------------------------------------------------- */ 
/* The data component of the messages varies with the sonarMessage as     */ 
/* defined by the structures below.                                       */ 
/* ---------------------------------------------------------------------- */ 

/* ---------------------------------------------------------------------- */ 
/* General integer parameter                                              */ 
/* ---------------------------------------------------------------------- */ 

typedef struct
{
  long value;                              /* Value for message           */ 
} SonarMessageLongType;


/* ---------------------------------------------------------------------- */ 
/* General purpose string argument.                                       */ 
/* ---------------------------------------------------------------------- */ 

typedef struct
{
  char name[SONAR_MESSAGE_STRING_LENGTH];  /* String parameter            */ 
} SonarMessageStringType;


/* ---------------------------------------------------------------------- */ 
/* SONAR_MESSAGE_STATUS parameters                                        */ 
/* ---------------------------------------------------------------------- */ 

typedef struct
{
  /* Incremented each time a ping is dropped.                             */ 
  unsigned long overflowCount; 

  /* The current error count.                                             */ 
  unsigned long errorCount;    

  /* Error ids of last 10 errors.                                         */ 
  short int lastError[10];     

  /* Disk space available for sonar data storage.                         */ 
  unsigned long freeDiskSpace;

  /* Indicates that data is being received (increments over time).        */ 
  unsigned long dataAcquisitionActivityIndicator;

  /* General service warning message                                      */ 
  unsigned char serviceNeeded;

  /* Temperature out of range                                             */ 
  unsigned char temperatureHigh;

  /* Humidity out of range indicator                                      */ 
  unsigned char humidityHigh;

  /* Reserved for additional health data                                  */ 
  unsigned char reserved;

} SonarMessageStatusType;


/* ---------------------------------------------------------------------- */ 
/* Ping First / Next data files                                           */ 
/* ---------------------------------------------------------------------- */ 

typedef struct
{
  /* Name used to identify this pulse for selection.                      */ 
  char fileName[FILE_NAME_SIZE];  

  /* Pulse description - null terminated ASCII string                     */ 
  char description[SONAR_MESSAGE_STRING_LENGTH];  

  /* Type of sonar pulse was designed for.                               */ 
  unsigned long int systemType;

  /* Type of pulse.                                                       */ 
  unsigned long int pulseType;

  /* Minimum frequency in Hz                                              */ 
  unsigned long int fMin;   

  /* Maximum frequency in Hz                                              */ 
  unsigned long int fMax;   

  /* Pulse duration in milliSeconds                                       */ 
  unsigned long int time;   

  /* Unique pulse identifier                                              */ 
  unsigned short int pulseID;    

} SonarMessagePingType;


/* ---------------------------------------------------------------------- */ 
/* Window for network data                                                */ 
/* ---------------------------------------------------------------------- */ 

typedef struct
{
  /* Frame decimation factor                                              */ 
  unsigned short frameDecimation; 

  /* Pixel decimation factor                                              */ 
  unsigned short decimation;      

  /* ADC samples to skip                                                  */ 
  unsigned long initialSkip;      

  /* Total samples to return                                              */ 
  /* If this value is set to zero, the maximum amount of data possible    */ 
  /* using the current ping rate or range will be sent.                   */ 
  /* SEG-Y limits the number of samples per packet to an unsigned short.  */ 
  /* If large sample sizes are required multiple packets must be used.    */ 
  unsigned long totalSamples;

  /* Packet size in samples                                               */ 
  /* Due to buffer limitations the current system retricts this to under  */ 
  /* 512KB.                                                               */ 
  unsigned long maxPacketSize;

} SonarMessageWindowType;

#endif  /* Not __SonarMessages_H__ */ 

/* ---------------------------------------------------------------------- */ 
/*                         end SonarMessages.h                            */ 
/* ---------------------------------------------------------------------- */ 
