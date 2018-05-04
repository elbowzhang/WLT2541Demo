#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include "OSAL.h"

#define RX_BUFF_SIZE    500

extern uint8 dataBuffer[RX_BUFF_SIZE];
extern uint16 dataBufferOffset ;
extern uint8 gapRole_TaskID;

extern void AT_Comand_Parser( uint8 port, uint8 event );
extern uint16 circular_add(uint16 x, uint16 y);
extern uint16 circular_diff(uint16 offset, uint16 tail);
extern unsigned long StringNumToUnsignedInteger(char *StringInteger);

#endif
