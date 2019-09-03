#ifndef __BATTERY
#define __BATTERY

#include "NL2020.h"

/* Slave Address for I2C Slave */
#define NL2020_ADR		 		0xB

void read_batter_and_display(void);
void I2C_Init(void);

#endif
