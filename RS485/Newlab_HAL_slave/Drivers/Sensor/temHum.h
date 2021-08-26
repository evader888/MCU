/*************************************************************************************************
  Filename:

 *****/


#ifndef _TEMHUM_H_
#define _TEMHUM_H_



#include "sys.h"

void hal_temHumInit(void);

void call_sht11(uint16_t *tem, uint16_t *hum);
//void connectionreset(void);

#endif	/* _HAL_TEMHUM_H_ */




