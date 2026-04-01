#ifndef ENCODER_GPIO_H
#define ENCODER_GPIO_H

#include <stdint.h>
#include <stdbool.h>

void EncoderGpio_init(void);
int32_t EncoderGpio_getAngle(void);

#endif
