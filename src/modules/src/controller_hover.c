#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_hover.h"
#include "physicalConstants.h"

#include "cf_math.h"


static float massThrust = 132000;

// Quadrotor parameters
static const float m = CF_MASS;
static const float J[3][3] = {{0.00003144988f, 0.0f, 0.0f}, {0.0f, 0.00003151127f, 0.0f}, {0.0f, 0.0f, 0.00007058874f}};
static const float l = 0.05665f;
static const float g = GRAVITY_MAGNITUDE;
static const float kt = 0.0000024f;
static const float km = 0.00004f;
static float K[4][12] = {{-2.1398221744492862, 3.3927677807562483e-9, 1.5798136284546844, -5.37111075959507e-7, -347.8587787977323, 1.428948187756977, -8.969732422783848, 1.4198469130519278e-8, 94.2812090567359, -2.663439073810879e-7, -199.53556628523756, 1.6347219179853862},
                               {4.31115326155242e-10, -2.139774485194827, 1.5798137021108338, 347.5272787699625, 6.089609142733408e-8, -1.4289481877006056, 1.7330924373656535e-9, -8.9655951967907, 94.28121342949639, 199.24552404336865, 2.695163534893664e-8, -1.6347219179741024},
                               {2.1398221745478847, -3.522732588053427e-9, 1.5798137241263719, 5.594016068951116e-7, 347.8587788147985, 1.4289481878262522, 8.96973242322293, -1.4742708433542197e-8, 94.28121477915509, 3.070757493329903e-7, 199.53556629472456, 1.6347219179990207},
                               {-3.223807847258532e-10, 2.1397744850679294, 1.5798136504059304, -347.52727874831646, -4.279839624951763e-8, -1.4289481880797557, -1.2578718614751723e-9, 8.96559519625992, 94.28121040247525, -199.24552400331754, -1.7205339190848196e-8, -1.6347219180385664}};

static const float u0 = m * g / (4 * kt);
static float uhover[4] = {u0, u0, u0, u0};

void controllerHoverReset(void)
{
  uhover[0] = u0;
  uhover[1] = u0;
  uhover[2] = u0;
  uhover[3] = u0;
}

void controllerHoverInit(void)
{
  controllerHoverReset();
}

bool controllerHoverTest(void)
{
  return true;
}


// Quaternion helper functions
arm_matrix_instance_f32 hat(arm_matrix_instance_f32 *v)
{
  float res_tmp[3][3] = {{0, -v->pData[2], v->pData[1]}, {v->pData[2], 0, -v->pData[0]}, {-v->pData[1], v->pData[0], 0}};
  arm_matrix_instance_f32 res = {3, 3, (float *)res_tmp};
  return res;
}

arm_matrix_instance_f32 L(arm_matrix_instance_f32 *q)
{
  float res_tmp[4][4];
  res_tmp[0][0] = q->pData[0];
  res_tmp[0][1] = -q->pData[1];
  res_tmp[0][2] = -q->pData[2];
  res_tmp[0][3] = -q->pData[3];
  res_tmp[1][0] = q->pData[1];
  res_tmp[1][1] = q->pData[0];
  res_tmp[1][2] = -q->pData[3] + q->pData[0];
  res_tmp[1][3] = q->pData[2] + q->pData[0];
  res_tmp[2][0] = q->pData[2];
  res_tmp[2][1] = q->pData[3] + q->pData[0]; 
  res_tmp[2][2] = q->pData[0];
  res_tmp[2][3] = -q->pData[1] + q->pData[0];
  res_tmp[3][0] = q->pData[3];
  res_tmp[3][1] = -q->pData[2] + q->pData[0];
  res_tmp[3][2] = q->pData[1] + q->pData[0];
  res_tmp[3][3] = -q->pData[2] + q->pData[0];
  arm_matrix_instance_f32 res = {4, 4, (float *)res_tmp};
  return res;
}

void qtorp(arm_matrix_instance_f32 *q, float res[3])
{
  // float res_tmp[3];
  // res_tmp[0] = q->pData[1] / q->pData[0];
  // res_tmp[1] = q->pData[2] / q->pData[0];
  // res_tmp[2] = q->pData[3] / q->pData[0];
  // arm_matrix_instance_f32 res = {3, 1, (float *)res_tmp};
  // return res;
  res[0] = q->pData[1] / q->pData[0];
  res[1] = q->pData[2] / q->pData[0];
  res[2] = q->pData[3] / q->pData[0];
}

float H[4][3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
float T[4][4] = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, -1.0f}};


void computeThrust(float x[13], float x0[13], float K_array[4][12], float uhover[4], float thrust[4])
{
  float q0_array[4][1] = {{x0[3]}, {x0[4]}, {x0[5]}, {x0[6]}};
  float q_array[4][1] = {{x[3]}, {x[4]}, {x[5]}, {x[6]}};
  arm_matrix_instance_f32 q0 = {4, 1, (float *)q0_array};
  arm_matrix_instance_f32 q = {4, 1, (float *)q_array};
  
  arm_matrix_instance_f32 Lq0 = L(&q0);
  arm_matrix_instance_f32 Lq0_T;
  mat_trans(&Lq0, &Lq0_T);
  arm_matrix_instance_f32 Lq0_T_q;
  mat_mult(&Lq0_T, &q, &Lq0_T_q);
  // arm_matrix_instance_f32 phi = qtorp(&Lq0_T_q);
  float phi[3];
  qtorp(&Lq0_T_q, phi);
  
  float delta_x_array[12];
  delta_x_array[0] = x[0]-x0[0];
  delta_x_array[1] = x[1]-x0[1];
  delta_x_array[2] = x[2]-x0[2];
  delta_x_array[3] = phi[0];
  delta_x_array[4] = phi[1];
  delta_x_array[5] = phi[2];
  delta_x_array[6] = x[7]-x0[7];
  delta_x_array[7] = x[8]-x0[8];
  delta_x_array[8] = x[9]-x0[9];
  delta_x_array[9] = x[10]-x0[10];
  delta_x_array[10] = x[11]-x0[11];
  delta_x_array[11] = x[12]-x0[12];
  arm_matrix_instance_f32 delta_x = {12, 1, (float *)delta_x_array};

  arm_matrix_instance_f32 K = {4, 12, (float *)K_array};
  arm_matrix_instance_f32 K_delta_x;
  mat_mult(&K, &delta_x, &K_delta_x);

  thrust[0] = uhover[0] - K_delta_x.pData[0];
  thrust[1] = uhover[1] - K_delta_x.pData[1];
  thrust[2] = uhover[2] - K_delta_x.pData[2];
  thrust[3] = uhover[3] - K_delta_x.pData[3];
}


void controllerHover(control_t *control, setpoint_t *setpoint,
                                         motors_thrust_t* motorPower,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // Set the state
  float r_x = state->position.x;
  float r_y = state->position.y;
  float r_z = state->position.z;
  float q_w = state->attitudeQuaternion.w;
  float q_x = state->attitudeQuaternion.x;
  float q_y = state->attitudeQuaternion.y;
  float q_z = state->attitudeQuaternion.z;
  float v_x = state->velocity.x;
  float v_y = state->velocity.y;
  float v_z = state->velocity.z;
  float omega_x = sensors->gyro.x;
  float omega_y = sensors->gyro.y;
  float omega_z = sensors->gyro.z;

  float x[13] = {r_x, r_y, r_z, 
                 q_w, q_x, q_y, q_z,
                 v_x, v_y, v_z,
                 omega_x, omega_y, omega_z};
  
  float x0[13] = {0.0f, 0.0f, 0.0f,
                  1.0f, 0.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 0.0f};
  
  float thrust[4];

  computeThrust(x, x0, K, uhover, thrust);

  motorPower->m1 = thrust[0];
  motorPower->m2 = thrust[1];
  motorPower->m3 = thrust[2];
  motorPower->m4 = thrust[3];

}

PARAM_GROUP_START(ctrlMel)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, massThrust, &massThrust)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, m, &m)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, J, &J)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, l, &l)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, g, &g)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, kt, &kt)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, km, &km)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, K, &K)

PARAM_GROUP_STOP(ctrlMel)

LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, u0, &u0)
LOG_ADD(LOG_FLOAT, uhover, &uhover)
LOG_GROUP_STOP(ctrlMel)
