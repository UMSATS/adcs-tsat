#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* ---------- public API --------------------------------------------------- */

/* Call once after the peripherals are initialised */
void AttitudeControl_Init(void);

/* Call as often as possible from the main while-loop (non-blocking) */
void AttitudeControl_Task(void);

#endif /* ATTITUDE_CONTROL_H */
