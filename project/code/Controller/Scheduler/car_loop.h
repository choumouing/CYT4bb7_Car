#include "zf_common_headfile.h"
#ifndef CAR_LOOP_H
#define CAR_LOOP_H



extern volatile uint8_t timer_10ms_flag;
extern volatile uint8_t timer_20ms_flag;
extern volatile uint8_t timer_40ms_flag;
extern volatile uint16 g_tick_1000HZ;

void car_loop_init(void);
void car_loop_poll(void);

#endif /* CAR_LOOP_H */
