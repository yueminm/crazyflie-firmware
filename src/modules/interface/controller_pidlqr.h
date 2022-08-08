#ifndef __CONTROLLER_PIDLQR_H__
#define __CONTROLLER_PIDLQR_H__

#include "stabilizer_types.h"

void controllerPIDLQRInit(void);
bool controllerPIDLQRTest(void);
void controllerPIDLQR(control_t *control, setpoint_t *setpoint,
                                         motors_thrust_t* motorPower,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_PIDLQR_H__