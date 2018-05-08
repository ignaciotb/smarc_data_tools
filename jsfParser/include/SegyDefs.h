/* ---------------------------------------------------------------------- */ 
/* SegyDefs.h                                                             */ 
/* ---------------------------------------------------------------------- */ 
/*                                                                        */ 
/* (c) Copyright 1997, 1998  EdgeTech,                                    */ 
/*                                                                        */ 
/* This file contains proprietary information, and trade secrets of       */ 
/* EdgeTech, and may not be disclosed or reproduced without the prior     */ 
/* written consent of EdgeTech.                                           */ 
/*                                                                        */ 
/* EdgeTech is not responsible for the consequences of the use or misuse  */ 
/* of this software, even if they result from defects in it.              */ 
/*                                                                        */ 
/* ---------------------------------------------------------------------- */ 
/*                                                                        */ 
/* EdgeTech SEG-Y subbottom data format description.                      */ 
/*                                                                        */ 
/* ---------------------------------------------------------------------- */ 

#ifndef __SEGYDEFS_H__
#define __SEGYDEFS_H__


/* ---------------------------------------------------------------------- */ 
/* Values for trigger type field                                          */ 
/* ---------------------------------------------------------------------- */ 

typedef enum
{
  TRIGGER_INTERNAL,           /* Internal trigger (0)                     */ 
  TRIGGER_EXTERNAL,           /* External trigger (1)                     */ 
} TriggerSourceType;


/* ---------------------------------------------------------------------- */ 
/* Each record of data has a 240 byte header, the content of which is     */ 
/* defined below.  The structure is intended to be compatible with most   */ 
/* implemented SEG-Y formats, as well as the original standard.           */ 
/*                                                                        */ 
/* Unused fields should be set to 0                                       */ 
/*                                                                        */ 
/* SEG-Y defined fields (** -> Highly Recommended in SEG-Y)               */ 
/* ---------------------------------------------------------------------- */ 

typedef struct 
{
  /* 0-3 : Trace Sequence Number (always 0) **                            */ 
  long int sequenceNumber; 

  /* 4-7 : Starting depth (window offset) in samples.                     */ 
  unsigned long int startDepth;    

  /* 8-11: Ping number (increments with ping) **                          */ 
  unsigned long int pingNum;        

  /* 12-15 : Channel Number (1 .. n) **                                   */ 
  unsigned long int channelNum;     

  /* 16-27                                                                */ 
  short int unused1[6];    

  /* 28-29 : ID Code (always 1 => seismic data) **                        */ 
  short int traceIDCode;   

  /* 30-33                                                                */ 
  short int unused2[2];    

  /* 34-35 : DataFormatType                                               */ 
  /*   0 = 1 short  per sample                                            */ 
  /*   1 = 2 shorts per sample, - stored as real(1), imag(1),             */ 
  /*   2 = 1 short  per sample  - before matched filter                   */ 
  /*   3 = 1 short  per sample  - real part analytic signal               */ 
  /*   4 = 1 short  per sample  - envelope data                           */ 
  short int dataFormat;    

  /* 36-37 : Distance from towfish to antannae in cm                      */ 
  short int NMEAantennaeR; 

  /* 38-39 : Dist to antannae starboard direction in cm                   */ 
  short int NMEAantannaeO; 
  
  /* 40-71 : Reserved for RS232 data - TBD                                */ 
  char RS232[32];          

  /* -------------------------------------------------------------------- */ 
  /* Navigation data :                                                    */ 
  /* If the coorUnits are seconds(2), the x values represent longitude    */ 
  /* and the y values represent latitude.  A positive value designates    */ 
  /* the number of seconds east of Greenwich Meridian or north of the     */ 
  /* equator.                                                             */ 
  /* -------------------------------------------------------------------- */ 

  /* 72-75 : Meters or Seconds of Arc                                     */ 
  long int sourceCoordX;   

  /* 76-79 : Meters or Seconds of Arc                                     */ 
  long int sourceCoordY;   

  /* 80-83 : mm or 10000 * (Minutes of Arc)                               */ 
  long int groupCoordX;    

  /* 84-87 : mm or 10000 * (Minutes of Arc)                               */ 
  long int groupCoordY;    

  /* 88-89 : Units of coordinates - 1->length (x /y), 2->seconds of arc   */ 
  short int coordUnits;    

  /* 90-113 : Annotation string                                           */ 
  char annotation[24];

  /* 114-115 : Samples in this packet **                                  */ 
  /* Note:  Large sample sizes require multiple packets.                  */ 
  unsigned short int samples;       

  /* 116-119 : Sample interval in ns of stored data **                    */ 
  unsigned long int sampleInterval;

  /* 120-121 : Gain factor of ADC                                         */ 
  unsigned short int ADCGain;       

  /* 122-123 : user pulse power setting (0 - 100) percent                 */ 
  short int pulsePower;    

  /* 124-125 : correlated data 1 - No, 2 - Yes                            */ 
  short int correlated;    

  /* 126-127 : Starting frequency in 10 * Hz                              */ 
  unsigned short int startFreq;     

  /* 128-129 : Ending frequency in 10 * Hz                                */ 
  unsigned short int endFreq;       

  /* 130-131 : Sweep length in ms                                         */ 
  unsigned short int sweepLength;   

  /* 132-139                                                              */ 
  short int unused7[4];    

  /* 140-141 : alias Frequency (sample frequency / 2)                     */ 
  unsigned short int aliasFreq;     

  /* 142-143 : Unique pulse identifier                                    */ 
  unsigned short int pulseID;    

  /* 144-155                                                              */ 
  short int unused8[6];    

  /* 156-157 : Year data recorded (CPU time)                              */ 
  short int year;          

  /* 158-159 :                                                            */ 
  short int day;           

  /* 160-161 :                                                            */ 
  short int hour;          

  /* 162-163 :                                                            */ 
  short int minute;        

  /* 164-165 :                                                            */ 
  short int second;        

  /* 166-167 : Always 3 (other not specified by standard)                 */ 
  short int timeBasis;     

  /* 168-169 :  -- defined as 2 -N volts for lsb                          */ 
  short int weightingFactor;

  /* 170-171 :                                                            */ 
  short int unused9;

  /* -------------------------------------------------------------------- */ 
  /* From pitch/roll/temp/heading sensor                                  */ 
  /* -------------------------------------------------------------------- */ 

  /* 172-173 : Compass heading                                            */ 
  short int heading;       

  /* 174-175 : Pitch                                                      */ 
  short int pitch;         

  /* 176-177 : Roll                                                       */ 
  short int roll;          

  /* 178-179 : Temperature                                                */ 
  short int temperature;   

  /* -------------------------------------------------------------------- */ 
  /* User defined area from 180-239                                       */ 
  /* -------------------------------------------------------------------- */ 

  /* 180-181 : Heave compensation offset (samples)                        */ 
  short int heaveCompensation;    

  /* 182-183 : TriggerSource (0 = internal, 1 = external)                 */ 
  short int trigSource;    

  /* 184-185 : Mark Number (0 = no mark)                                  */ 
  unsigned short int markNumber;    

  /* 186-187 :                                                            */ 
  short int NMEAHour;

  /* 188-189 :                                                            */ 
  short int NMEAMinutes;

  /* 190-191 :                                                            */ 
  short int NMEASeconds;

  /* 192-193 :                                                            */ 
  short int NMEACourse;

  /* 194-195 :                                                            */ 
  short int NMEASpeed;

  /* 196-197 :                                                            */ 
  short int NMEADay;

  /* 198-199 :                                                            */ 
  short int NMEAYear;

  /* 200-203 : Millieconds today                                          */ 
  unsigned long int millisecondsToday;

  /* 204-205 : Maximum absolute value for ADC samples for this packet     */ 
  unsigned short int ADCMax;        

  /* 206-207 : System constant in tenths of a dB                          */ 
  short int calConst;      

  /* 208-209 : Vehicle ID                                                 */ 
  short int vehicleID;     

  /* 210-215 : Software version number                                    */ 
  char softwareVersion[6]; 

  /* Following items are not in X-Star                                    */ 

  /* 216-219 : Initial spherical correction factor (useful for multiping /*/ 
  /* deep application) * 100                                              */ 
  long int sphericalCorrection;
  
  /* 220-221 : 1-N (Each ping starts with packet 1)                       */ 
  unsigned short int packetNum;     

  /* 222-223 : A/D decimation before FFT                                  */ 
  short int ADCDecimation; 

  /* 224-225 : Decimation factor after FFT                                */ 
  short int decimation;    

  /* 226-239                                                              */ 
  short int unuseda[7];

  /* -------------------------------------------------------------------- */ 
  /* Data area begins here                                                */ 
  /* -------------------------------------------------------------------- */ 
  /* Data begins at byte 240, has this.samples points in it               */ 
  /* short int data[];                                                    */ 
  /* -------------------------------------------------------------------- */ 
} SegyDataType;


/* ---------------------------------------------------------------------- */ 

#endif  /* end Not __SEGYDEFS_H__ */ 

/* ---------------------------------------------------------------------- */ 
/*                            end SegyDefs.h                              */ 
/* ---------------------------------------------------------------------- */ 