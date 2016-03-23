#ifndef __GPIO16_H__
#define __GPIO16_H__

#include "user_config.h"
#ifndef MakeUSER2

void gpio16_output_conf(void);
void gpio16_output_set(uint8 value);
void gpio16_input_conf(void);
uint8 gpio16_input_get(void);

#endif
#endif
