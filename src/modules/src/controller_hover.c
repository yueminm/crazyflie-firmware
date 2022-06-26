#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_hover.h"
#include "physicalConstants.h"
#include <stdio.h>

#include "cf_math.h"
#include "console.h"



// Quadrotor parameters
static const float m = CF_MASS;
static const float J[3][3] = {{0.00003144988f, 0.0f, 0.0f}, {0.0f, 0.00003151127f, 0.0f}, {0.0f, 0.0f, 0.00007058874f}};
// static const float l = 0.042f;
static const float g = GRAVITY_MAGNITUDE;
// static const float kt = 0.000009f;
static float K[4][12] = {{-2.1111338809910256, -7.207516174425559e-10, 1.5785755111434507, 6.017379686544216e-8, -211.97035532655272, 1.5610626124198432, -7.076254625482424, -2.254841761000844e-9, 48.7368361888653, 1.974425853122233e-8, -93.3604575689346, 3.7303351206875837},
                         {-8.795079714807122e-13, -2.1110711123990322, 1.5785755142261677, 211.77143487247827, 1.3160658601897058e-9, -1.5610626123239035, 2.918253647663058e-11, -7.0731146868181565, 48.736836283475604, 93.22539702517304, 6.68246173887247e-10, -3.7303351204756448},
                         {2.1111338809797546, 8.039592555668976e-10, 1.5785755227341753, -6.68554050895323e-8, 211.9703553283179, 1.5610626124157603, 7.076254625509039, 2.495118152731056e-9, 48.73683655147123, -2.2637825756993564e-8, 93.36045756999391, 3.7303351206817412},
                         {2.1111338809797546, 8.039592555668976e-10, 1.5785755227341753, -6.68554050895323e-8, 211.9703553283179, 1.5610626124157603, 7.076254625509039, 2.495118152731056e-9, 48.73683655147123, -2.2637825756993564e-8, 93.36045756999391, 3.7303351206817412}};


static const float u0 = m / 0.06f * 65536.0f / 4;
static float uhover[4] = {u0, u0, u0, u0};
static float thrust_new[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float x0[13] = {0.0f, 0.0f, 1.0f,
                      1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f};

static float x[13] = {0.0f, 0.0f, 0.0f,
                      1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f};

static float velocity_world[3] = {0.0f, 0.0f, 0.0f};

// Debugging
// static int count = 0;
// static float error_point[10][13];



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
  assert_aligned_4_bytes(v);
  float res_tmp[3][3] = {{0, -v->pData[2], v->pData[1]}, {v->pData[2], 0, -v->pData[0]}, {-v->pData[1], v->pData[0], 0}};
  arm_matrix_instance_f32 res = {3, 3, (float *)res_tmp};
  return res;
}

arm_matrix_instance_f32 L(arm_matrix_instance_f32 *q)
{
  assert_aligned_4_bytes(q);
  float res_tmp[4][4];
  res_tmp[0][0] = q->pData[0];
  res_tmp[0][1] = -q->pData[1];
  res_tmp[0][2] = -q->pData[2];
  res_tmp[0][3] = -q->pData[3];
  res_tmp[1][0] = q->pData[1];
  res_tmp[1][1] = q->pData[0];
  res_tmp[1][2] = -q->pData[3];
  res_tmp[1][3] = q->pData[2];
  res_tmp[2][0] = q->pData[2];
  res_tmp[2][1] = q->pData[3]; 
  res_tmp[2][2] = q->pData[0];
  res_tmp[2][3] = -q->pData[1];
  res_tmp[3][0] = q->pData[3];
  res_tmp[3][1] = -q->pData[2];
  res_tmp[3][2] = q->pData[1];
  res_tmp[3][3] = q->pData[0];
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
  assert_aligned_4_bytes(q);
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
  float Lq0_T_array[4][4];
  arm_matrix_instance_f32 Lq0_T = {4, 4, (float *)Lq0_T_array};
  mat_trans(&Lq0, &Lq0_T);
  float Lq0_T_q_array[4][1];
  arm_matrix_instance_f32 Lq0_T_q = {4, 1, (float *)Lq0_T_q_array};
  mat_mult(&Lq0_T, &q, &Lq0_T_q);
  // arm_matrix_instance_f32 phi = qtorp(&Lq0_T_q);
  float phi[3];
  qtorp(&Lq0_T_q, phi);
  
  float32_t delta_x_array[12];
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
  float K_delta_x_array[4][1];
  arm_matrix_instance_f32 K_delta_x = {4, 1, (float *)K_delta_x_array};
  mat_mult(&K, &delta_x, &K_delta_x);

  thrust[0] = (uhover[0] - K_delta_x.pData[0])*1.0f;
  thrust[1] = (uhover[1] - K_delta_x.pData[1])*1.0f;
  thrust[2] = (uhover[2] - K_delta_x.pData[2])*1.0f;
  thrust[3] = (uhover[3] - K_delta_x.pData[3])*1.0f;
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

  if (setpoint->velocity.z == 0) {
    motorPower->m1 = 0;
    motorPower->m2 = 0;
    motorPower->m3 = 0;
    motorPower->m4 = 0;
  }

  else {
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
    float omega_x = radians(sensors->gyro.x);
    float omega_y = radians(sensors->gyro.y);
    float omega_z = radians(sensors->gyro.z);

    velocity_world[0] = state->velocityWorld.x;
    velocity_world[1] = state->velocityWorld.y;
    velocity_world[2] = state->velocityWorld.z;

    x[0] = r_x;
    x[1] = r_y;
    x[2] = r_z;
    x[3] = q_w;
    x[4] = q_x;
    x[5] = q_y;
    x[6] = q_z;
    x[7] = v_x;
    x[8] = v_y;
    x[9] = v_z;
    x[10] = omega_x;
    x[11] = omega_y;
    x[12] = omega_z;
   
    // float x0[13] = {0.0f, 0.0f, 0.0f,
    //                 1.0f, 0.0f, 0.0f, 0.0f,
    //                 0.0f, 0.0f, 0.0f,
    //                 0.0f, 0.0f, 0.0f};
    
    

    computeThrust(x, x0, K, uhover, thrust_new);
    
    if (!(isnan(thrust_new[0]) || isnan(thrust_new[1]) || isnan(thrust_new[2]) || isnan(thrust_new[3]) ||
        thrust_new[0] > 65536 || thrust_new[1] > 65536 || thrust_new[2] > 65536 || thrust_new[3] > 65536 ||
        thrust_new[0] < 0 || thrust_new[1] < 0 || thrust_new[2] < 0 || thrust_new[3] < 0))
    {
      int thrust1 = (int) thrust_new[0] * 2.0;
      int thrust2 = (int) thrust_new[1] * 2.0;
      int thrust3 = (int) thrust_new[2] * 2.0;
      int thrust4 = (int) thrust_new[3] * 2.0;

      motorPower->m1 = thrust1;
      motorPower->m2 = thrust2;
      motorPower->m3 = thrust3;
      motorPower->m4 = thrust4;
    }

    // if (isnan(thrust_new[0]) || isnan(thrust_new[1]) || isnan(thrust_new[2]) || isnan(thrust_new[3]))
    // {
    //   if (count > 9) {return;}
    //   error_point[count][0] = r_x;
    //   error_point[count][1] = r_y;
    //   error_point[count][2] = r_z;
    //   error_point[count][3] = q_w;
    //   error_point[count][4] = q_x;
    //   error_point[count][5] = q_y;
    //   error_point[count][6] = q_z;
    //   error_point[count][7] = v_x;
    //   error_point[count][8] = v_y;
    //   error_point[count][9] = v_z;
    //   error_point[count][10] = omega_x;
    //   error_point[count][11] = omega_y;
    //   error_point[count][12] = omega_z;
    //   count = count + 1;
    // }
  }
}

PARAM_GROUP_START(ctrlHover)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, m, &m)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, J, &J)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, l, &l)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, g, &g)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, kt, &kt)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, km, &km)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, K, &K)

PARAM_GROUP_STOP(ctrlHover)

LOG_GROUP_START(ctrlHover)
LOG_ADD(LOG_FLOAT, thrust1, &thrust_new[0])
LOG_ADD(LOG_FLOAT, thrust2, &thrust_new[1])
LOG_ADD(LOG_FLOAT, thrust3, &thrust_new[2])
LOG_ADD(LOG_FLOAT, thrust4, &thrust_new[3])

LOG_ADD(LOG_FLOAT, rx0, &x0[0])
LOG_ADD(LOG_FLOAT, ry0, &x0[1])
LOG_ADD(LOG_FLOAT, rz0, &x0[2])

LOG_ADD(LOG_FLOAT, qw0, &x0[3])
LOG_ADD(LOG_FLOAT, qx0, &x0[4])
LOG_ADD(LOG_FLOAT, qy0, &x0[5])
LOG_ADD(LOG_FLOAT, qz0, &x0[6])

LOG_ADD(LOG_FLOAT, vx0, &x0[7])
LOG_ADD(LOG_FLOAT, vy0, &x0[8])
LOG_ADD(LOG_FLOAT, vz0, &x0[9])

LOG_ADD(LOG_FLOAT, ox0, &x0[10])
LOG_ADD(LOG_FLOAT, oy0, &x0[11])
LOG_ADD(LOG_FLOAT, oz0, &x0[12])

LOG_ADD(LOG_FLOAT, rx, &x[0])
LOG_ADD(LOG_FLOAT, ry, &x[1])
LOG_ADD(LOG_FLOAT, rz, &x[2])

LOG_ADD(LOG_FLOAT, qw, &x[3])
LOG_ADD(LOG_FLOAT, qx, &x[4])
LOG_ADD(LOG_FLOAT, qy, &x[5])
LOG_ADD(LOG_FLOAT, qz, &x[6])

LOG_ADD(LOG_FLOAT, vx, &x[7])
LOG_ADD(LOG_FLOAT, vy, &x[8])
LOG_ADD(LOG_FLOAT, vz, &x[9])

LOG_ADD(LOG_FLOAT, ox, &x[10])
LOG_ADD(LOG_FLOAT, oy, &x[11])
LOG_ADD(LOG_FLOAT, oz, &x[12])

LOG_ADD(LOG_FLOAT, world_vx, &velocity_world[0])
LOG_ADD(LOG_FLOAT, world_vy, &velocity_world[1])
LOG_ADD(LOG_FLOAT, world_vz, &velocity_world[2])
// LOG_ADD(LOG_FLOAT, err1_rx, &error_point[0][0])
// LOG_ADD(LOG_FLOAT, err1_ry, &error_point[0][1])
// LOG_ADD(LOG_FLOAT, err1_rz, &error_point[0][2])

// LOG_ADD(LOG_FLOAT, err1_qw, &error_point[0][3])
// LOG_ADD(LOG_FLOAT, err1_qx, &error_point[0][4])
// LOG_ADD(LOG_FLOAT, err1_qy, &error_point[0][5])
// LOG_ADD(LOG_FLOAT, err1_qz, &error_point[0][6])

// LOG_ADD(LOG_FLOAT, err1_vx, &error_point[0][7])
// LOG_ADD(LOG_FLOAT, err1_vy, &error_point[0][8])
// LOG_ADD(LOG_FLOAT, err1_vz, &error_point[0][9])

// LOG_ADD(LOG_FLOAT, err1_ox, &error_point[0][10])
// LOG_ADD(LOG_FLOAT, err1_oy, &error_point[0][11])
// LOG_ADD(LOG_FLOAT, err1_oz, &error_point[0][12])

// LOG_ADD(LOG_FLOAT, err5_rx, &error_point[5][0])
// LOG_ADD(LOG_FLOAT, err5_ry, &error_point[5][1])
// LOG_ADD(LOG_FLOAT, err5_rz, &error_point[5][2])

// LOG_ADD(LOG_FLOAT, err5_qw, &error_point[5][3])
// LOG_ADD(LOG_FLOAT, err5_qx, &error_point[5][4])
// LOG_ADD(LOG_FLOAT, err5_qy, &error_point[5][5])
// LOG_ADD(LOG_FLOAT, err5_qz, &error_point[5][6])

// LOG_ADD(LOG_FLOAT, err5_vx, &error_point[5][7])
// LOG_ADD(LOG_FLOAT, err5_vy, &error_point[5][8])
// LOG_ADD(LOG_FLOAT, err5_vz, &error_point[5][9])

// LOG_ADD(LOG_FLOAT, err5_ox, &error_point[5][10])
// LOG_ADD(LOG_FLOAT, err5_oy, &error_point[5][11])
// LOG_ADD(LOG_FLOAT, err5_oz, &error_point[5][12])

LOG_GROUP_STOP(ctrlHover)
