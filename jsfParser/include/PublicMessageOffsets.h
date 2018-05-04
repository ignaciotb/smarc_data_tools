/* ---------------------------------------------------------------------- */ 
/* Begin PublicMessageOffsets.h                                           */ 
/*                                                                        */ 
/* ---------------------------------------------------------------------- */ 
/*                                                                        */ 
/* (c) Copyright 1998  EdgeTech,                                          */ 
/*                                                                        */ 
/* This file contains proprietary information, and trade secrets of       */ 
/* EdgeTech, and may not be disclosed or reproduced without the prior     */ 
/* written consent of EdgeTech.                                           */ 
/*                                                                        */ 
/* EdgeTech is not responsible for the consequences of the use or misuse  */ 
/* of this software, even if they result from defects in it.              */ 
/*                                                                        */ 
/* Offsets for public message types.                                      */ 
/* ---------------------------------------------------------------------- */ 

#ifndef __PUBLIC_MESSAGE_OFFSETS_H__
#define __PUBLIC_MESSAGE_OFFSETS_H__


/* ---------------------------------------------------------------------- */ 
/* Message offsets for each message.                                      */ 
/* ---------------------------------------------------------------------- */ 

/* Starting message offset for standard messages                          */ 
#define MESSAGE_OFFSET_STANDARD               (20)

/* Starting message offset for standard status messages                   */ 
#define MESSAGE_OFFSET_STATUS                 (40)

/* Starting message offset for standard data messages                     */ 
#define MESSAGE_OFFSET_DATA                   (80)

/* Starting message offset for standard processing messages               */ 
#define MESSAGE_OFFSET_PROCESS               (100)

/* Starting message offset for standard pulse messages                    */ 
#define MESSAGE_OFFSET_PULSES                (120)

/* Starting message offset for standard ADC messages                      */ 
#define MESSAGE_OFFSET_ADC                   (140)


/* ---------------------------------------------------------------------- */ 

#endif  /* Not __PUBLIC_MESSAGE_OFFSETS_H__ */ 

/* ---------------------------------------------------------------------- */ 
/*                  end PublicMessageOffsets.h                            */ 
/* ---------------------------------------------------------------------- */ 