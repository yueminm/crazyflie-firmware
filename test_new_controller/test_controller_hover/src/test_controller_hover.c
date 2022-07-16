#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include <math.h>

#define DEBUG_MODULE "TESTCONTROLLERHOVER"



static void setInitialSetpoint(setpoint_t *setpoint, float vz)
{
  setpoint->mode.z = modeAbs;

  setpoint->mode.yaw = modeVelocity;

  setpoint->velocity.z = vz;

  setpoint->velocity_body = true;
}

static void setMotorPower(motors_thrust_t *motorPower, int thrust[4])
{
  motorPower->m1 = thrust[0];
  motorPower->m2 = thrust[1];
  motorPower->m3 = thrust[2];
  motorPower->m4 = thrust[3];
}

// typedef enum {
//     idle,
//     hover
// } State;

// static State state = idle;

static int startFlight = 0;

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

void appMain()
{
  static setpoint_t setpoint;
  static motors_thrust_t motorPower;
  int thrust_idle[4] = {0, 0, 0, 0};


  vTaskDelay(M2T(1000));


  // Getting Param IDs of the deck driver initialization
  // paramVarId_t idLocoPositioningDeck = paramGetVarId("deck", "bcDWM1000");
 
 
  // DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(10));
    // DEBUG_PRINT("first \n");

    // Check if decks are properly mounted
    // uint8_t locoPositioningInit = paramGetUint(idLocoPositioningDeck);

    if (startFlight == 0) {
      setInitialSetpoint(&setpoint, 0.0f);
      setMotorPower(&motorPower, thrust_idle);
      }

    else if (startFlight == 1) {
      setInitialSetpoint(&setpoint, 1.0f);
      commanderSetSetpoint(&setpoint, 3);
      }

    else if (startFlight == 2) {
      setInitialSetpoint(&setpoint, 2.0f);
      commanderSetSetpoint(&setpoint, 3);
      }
  }
}


/**
 * [Documentation for the activation group ...]
 */
PARAM_GROUP_START(activation)

/**
 * @brief [Documentation for the parameter below ...]
 */
PARAM_ADD_CORE(PARAM_INT32, startFlight, &startFlight)

PARAM_GROUP_STOP(flightActivation)