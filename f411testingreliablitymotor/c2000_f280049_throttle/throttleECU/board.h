#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

void Board_initHW(void);
void Board_digitalRelay(uint16_t on);
void Board_digitalEnables(uint16_t on);

uint32_t Board_millis(void);
uint32_t Board_cycleCounter(void);
uint32_t Board_cyclesToUs(uint32_t deltaCycles);

#endif
