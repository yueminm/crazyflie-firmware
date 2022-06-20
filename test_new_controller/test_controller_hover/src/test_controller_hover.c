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

#define DEBUG_MODULE "CONTROLLERHOVER"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;

static State state = idle;


#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

void appMain()
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));


  // Getting Param IDs of the deck driver initialization
  paramVarId_t idLocoPositioningDeck = paramGetVarId("deck", "bcDWM1000");
 
  // Intialize the setpoint structure
  setpoint_t setpoint;
 
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(10));
    DEBUG_PRINT("first \n");

    // Check if decks are properly mounted
    uint8_t locoPositioningInit = paramGetUint(idLocoPositioningDeck);

    if (1) {
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, 1.0f, 0.0f);
        commanderSetSetpoint(&setpoint, 3);
      }
    
      DEBUG_PRINT("second \n");
    
    DEBUG_PRINT("third \n");
  }
}
