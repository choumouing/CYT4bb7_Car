#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"


extern volatile uint8_t timer_100HZ_flag;
extern volatile uint8_t timer_50HZ_flag;
extern volatile uint8_t timer_25HZ_flag;
extern volatile uint16 g_tick_1000HZ;

extern float car_forward_target;
extern float car_strafe_target;
extern float car_rotate_target;
extern uint8 car_control_enabled;
extern uint8 car_emergency_stop_active;

void car_loop_init(void);
void car_loop_poll(void);

#endif /* CAR_LOOP_H */
