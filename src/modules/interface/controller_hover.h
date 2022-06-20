#ifndef __CONTROLLER_HOVER_H__
#define __CONTROLLER_HOVER_H__

#include "stabilizer_types.h"

void controllerHoverInit(void);
bool controllerHoverTest(void);
void controllerHover(control_t *control, setpoint_t *setpoint,
                                         motors_thrust_t* motorPower,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_HOVER_H__