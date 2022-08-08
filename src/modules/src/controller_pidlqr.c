
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pidlqr.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

// Quadrotor parameters
static const float m = CF_MASS;
static const float J[3][3] = {{0.00003144988f, 0.0f, 0.0f}, {0.0f, 0.00003151127f, 0.0f}, {0.0f, 0.0f, 0.00007058874f}};
// static const float l = 0.042f;
static const float g = GRAVITY_MAGNITUDE;
// static const float kt = 0.000009f;
// static float K[4][12] = {{-2.1111338809910256, -7.207516174425559e-10, 1.5785755111434507, 6.017379686544216e-8, -211.97035532655272, 1.5610626124198432, -7.076254625482424, -2.254841761000844e-9, 48.7368361888653, 1.974425853122233e-8, -93.3604575689346, 3.7303351206875837},
//                          {-8.795079714807122e-13, -2.1110711123990322, 1.5785755142261677, 211.77143487247827, 1.3160658601897058e-9, -1.5610626123239035, 2.918253647663058e-11, -7.0731146868181565, 48.736836283475604, 93.22539702517304, 6.68246173887247e-10, -3.7303351204756448},
//                          {2.1111338809797546, 8.039592555668976e-10, 1.5785755227341753, -6.68554050895323e-8, 211.9703553283179, 1.5610626124157603, 7.076254625509039, 2.495118152731056e-9, 48.73683655147123, -2.2637825756993564e-8, 93.36045756999391, 3.7303351206817412},
//                          {-1.0530694218930361e-11, 2.1110711125049213, 1.5785755196639806, -211.77143488135056, 4.338088604067049e-10, -1.5610626124084546, -2.9975189299189435e-12, 7.073114687133112, 48.73683645718751, -93.22539702894497, 3.7912434828436544e-10, -3.730335120613992}};

// static const float u0 = m / 0.06f * 65536.0f / 4;

static float K[4][12] = {{-20194.973476459825, 20234.192401598153, 183794.8335813481, -54447.741290896396, -54342.09914007165, 0.05843962720078665, -12355.27968211007, 12379.282893484005, 68753.23259533918, -3262.627220560879, -3256.291484918947, 0.05990403111962938},
                         {20194.97347415466, 20234.192401948847, 183794.83358357497, -54447.74128776482, 54342.09913674533, -0.05844383981804879, 12355.279680969881, 12379.28289319767, 68753.2325955883, -3262.627220516521, 3256.2914847655456, -0.05990410509617307},
                         {20194.973473644684, -20234.192401524433, 183794.83356956573, 54447.74128033818, 54342.09913557802, 0.05844219836279859, 12355.27968074988, -12379.282891984742, 68753.23258888074, 3262.627220404553, 3256.291484662595, 0.059904067097588734},
                         {-20194.973476969768, -20234.192401875112, 183794.8335673389, 54447.74127720657, -54342.09914123886, -0.058446373402984524, -12355.279682330045, -12379.282891698396, 68753.23258863162, 3262.6272203601934, -3256.2914850218935, -0.059904136441328665}};

// static const float u0 = m / (0.06f / 4) * 65536.0f / 4;
static const float u0 = 36000.0f;
static float uhover[4] = {u0, u0, u0, u0};
static float thrust_new[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float x0[13] = {0.0f, 0.0f, 0.2f,
                       1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f};

static float x[13] = {0.0f, 0.0f, 0.0f,
                      1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f};


void controllerPIDLQRInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  uhover[0] = u0;
  uhover[1] = u0;
  uhover[2] = u0;
  uhover[3] = u0;
}

bool controllerPIDLQRTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
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
  delta_x_array[0] = x[0] - x0[0];
  delta_x_array[1] = x[1] - x0[1];
  delta_x_array[2] = x[2] - x0[2];
  delta_x_array[3] = phi[0];
  delta_x_array[4] = phi[1];
  delta_x_array[5] = phi[2];
  delta_x_array[6] = x[7] - x0[7];
  delta_x_array[7] = x[8] - x0[8];
  delta_x_array[8] = x[9] - x0[9];
  delta_x_array[9] = x[10] - x0[10];
  delta_x_array[10] = x[11] - x0[11];
  delta_x_array[11] = x[12] - x0[12];
  arm_matrix_instance_f32 delta_x = {12, 1, (float *)delta_x_array};

  arm_matrix_instance_f32 K = {4, 12, (float *)K_array};
  float K_delta_x_array[4][1];
  arm_matrix_instance_f32 K_delta_x = {4, 1, (float *)K_delta_x_array};
  mat_mult(&K, &delta_x, &K_delta_x);

  thrust[0] = (uhover[0] - K_delta_x.pData[0] * 1.0f) * 1.0f;
  thrust[1] = (uhover[1] - K_delta_x.pData[1] * 1.0f) * 1.0f;
  thrust[2] = (uhover[2] - K_delta_x.pData[2] * 1.0f) * 1.0f;
  thrust[3] = (uhover[3] - K_delta_x.pData[3] * 1.0f) * 1.0f;
}

void controllerPIDLQR(control_t *control, setpoint_t *setpoint,
                      motors_thrust_t *motorPower,
                      const sensorData_t *sensors,
                      const state_t *state,
                      const uint32_t tick)

{
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

  // velocity_world[0] = state->velocityWorld.x;
  // velocity_world[1] = state->velocityWorld.y;
  // velocity_world[2] = state->velocityWorld.z;

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


  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);
       
      #ifdef YAW_MAX_DELTA
      float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
      // keep the yaw setpoint within +/- YAW_MAX_DELTA from the current yaw
        if (delta > YAW_MAX_DELTA)
        {
          attitudeDesired.yaw = state->attitude.yaw + YAW_MAX_DELTA;
        }
        else if (delta < -YAW_MAX_DELTA)
        {
          attitudeDesired.yaw = state->attitude.yaw - YAW_MAX_DELTA;
        }
      #endif
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)


LOG_GROUP_START(ctrlPIDLQR)
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
LOG_GROUP_STOP(ctrlPIDLQR)