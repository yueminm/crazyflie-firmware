#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
// #include "controller_pid.h"
// #include "controller_mellinger.h"
// #include "controller_indi.h"
// #include "controller_hover.h"
#include "controller_pidlqr.h"

#include "autoconf.h"

// #define DEFAULT_CONTROLLER ControllerTypeHover
#define DEFAULT_CONTROLLER ControllerTypePIDLQR
static ControllerType currentController = ControllerTypeAny;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, setpoint_t *setpoint, motors_thrust_t* motorPower, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} ControllerFcns;

static ControllerFcns controllerFunctions[] = {
    {.init = 0, .test = 0, .update = 0, .name = "None"}, // Any
    // {.init = controllerPidInit, .test = controllerPidTest, .update = controllerPid, .name = "PID"},
    // {.init = controllerMellingerInit, .test = controllerMellingerTest, .update = controllerMellinger, .name = "Mellinger"},
    // {.init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI, .name = "INDI"},
    // {.init = controllerHoverInit, .test = controllerHoverTest, .update = controllerHover, .name = "Hover"},
    // {.init = controllerPIDLQRInit, .test = controllerPIDLQRTest, .update = controllerPIDLQR, .name = "PIDLQR"},
    {.init = controllerPIDLQRInit},

};

void controllerInit(ControllerType controller) {
  if (controller < 0 || controller >= ControllerType_COUNT) {
    return;
  }

  currentController = controller;

  if (ControllerTypeAny == currentController) {
    currentController = DEFAULT_CONTROLLER;
  }

  // #if defined(CONFIG_CONTROLLER_PID)
  //   #define CONTROLLER ControllerTypePID
  // #elif defined(CONFIG_CONTROLLER_INDI)
  //   #define CONTROLLER ControllerTypeINDI
  // #elif defined(CONFIG_CONTROLLER_MELLINGER)
  //   #define CONTROLLER ControllerTypeMellinger
  // #if defined(CONFIG_CONTROLLER_Hover)
  //   #define CONTROLLER ControllerTypeHover
#if defined(CONFIG_CONTROLLER_PIDLQR)
  #define CONTROLLER ControllerTypePIDLQR
#else
  #define CONTROLLER ControllerTypeAny
#endif

  ControllerType forcedController = CONTROLLER;
  if (forcedController != ControllerTypeAny) {
    DEBUG_PRINT("Controller type forced\n");
    currentController = forcedController;
  }

  initController();

  DEBUG_PRINT("Using %s (%d) controller\n", controllerGetName(), currentController);
}

ControllerType getControllerType(void) {
  return currentController;
}

static void initController() {
  controllerFunctions[currentController].init();
}

bool controllerTest(void) {
  return controllerFunctions[currentController].test();
}

void controller(control_t *control, setpoint_t *setpoint, motors_thrust_t* motorPower, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerFunctions[currentController].update(control, setpoint, motorPower, sensors, state, tick);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}
