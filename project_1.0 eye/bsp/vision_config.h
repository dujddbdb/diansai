#ifndef __VISION_CONFIG_H__
#define __VISION_CONFIG_H__

#include "stm32f4xx.h"

/* PID parameters */
#define PID_ERROR_BLEND_START          5.0f
#define PID_ERROR_BLEND_END            60.0f
#define PID_GAIN_SLEW_FACTOR           0.22f

#define PID_KP_LARGE                   10.0f
#define PID_KI_LARGE                   0.0f
#define PID_KD_LARGE                   100.1f

#define PID_KP_SMALL                   0.1f
#define PID_KI_SMALL                   0.0f
#define PID_KD_SMALL                   50.1f

#define PID_INT_SEPARATION_THRESHOLD   500
#define GIMBAL_PIXEL_DEADBAND          1.01f
#define GIMBAL_INTEGRAL_LIMIT_DEG      0.01f
#define PID_OUTPUT_MAX_X               1.0f
#define PID_OUTPUT_MAX_Y               1.0f
#define PIXEL_TO_ANGLE_FACTOR          0.001f
#define GIMBAL_DERIVATIVE_FILTER_ALPHA 0.35f
#define GIMBAL_OUTPUT_SLEW_DEG         0.65f

/* Stepper configuration */
#define GIMBAL_ADDR_X                  0x01
#define GIMBAL_ADDR_Y                  0x01
#define GIMBAL_RPM_DEFAULT             1.0
#define GIMBAL_ACC_DEFAULT             1
#define GIMBAL_PULSES_PER_REV          3200.0f
#define GIMBAL_DEGREES_PER_REV         360.0f
#define GIMBAL_PULSES_PER_DEG          (GIMBAL_PULSES_PER_REV / GIMBAL_DEGREES_PER_REV)

/* IMU feedforward */
#define GIMBAL_COMPENSATION_FACTOR     0.55f
#define GIMBAL_YAW_DELTA_THRESHOLD     0.0f
#define GIMBAL_PITCH_DELTA_THRESHOLD   0.0f
#define GIMBAL_RATE_FEEDFORWARD_FACTOR 0.045f
#define GIMBAL_FEEDFORWARD_MAX_DEG     0.80f

#define VISION_IMU_FEEDFORWARD_PERIOD_MS 1U
#define VISION_IMU_DRAIN_MAX_PACKETS     6U
#define VISION_CONTROL_PERIOD_MS         5U
#define VISION_IMU_FEEDFORWARD_ALWAYS_ON 1U

#define VISION_IMU_KALMAN_ANGLE_Q      0.002f
#define VISION_IMU_KALMAN_RATE_Q       0.10f
#define VISION_IMU_KALMAN_BIAS_Q       0.00008f
#define VISION_IMU_KALMAN_MEAS_R       0.90f
#define VISION_IMU_KALMAN_RATE_DECAY   0.996f
#define VISION_IMU_KALMAN_Q            VISION_IMU_KALMAN_ANGLE_Q
#define VISION_IMU_KALMAN_R            VISION_IMU_KALMAN_MEAS_R
#define VISION_IMU_DELTA_LIMIT_DEG     3.0f
#define VISION_IMU_BIAS_LIMIT_DEG      8.0f
#define VISION_IMU_RATE_LIMIT_DPS      720.0f
#define VISION_IMU_RATE_EPS_DPS        0.001f
#define VISION_IMU_MIN_CORNER_BLEND    0.08f
#define VISION_IMU_DT_MIN_MS           1U
#define VISION_IMU_DT_MAX_MS           40U

/* Laser trigger */
#define VISION_LASER_THRESHOLD_PX      30
#define VISION_LASER_HOLD_FRAMES       3

/* Runtime motor speed */
#define VISION_GIMBAL_RPM_MIN          20.0f
#define VISION_GIMBAL_RPM_MAX          120.0f
#define VISION_GIMBAL_RPM_SLEW         0.25f
#define VISION_CORNER_RPM_REDUCE       0.35f

/* Circle scan */
#define VISION_CIRCLE_RADIUS_PX        26.0f
#define VISION_CIRCLE_STEP_RAD         0.12f

/* ROI scan strategy */
#define VISION_SCAN_X_MIN              (-30.0f)
#define VISION_SCAN_X_MAX              ( 30.0f)
#define VISION_SCAN_Y_MIN              (-15.0f)
#define VISION_SCAN_Y_MAX              ( 15.0f)
#define VISION_SCAN_SPEED              (5.0f)

#define VISION_LOCK_FRAMES             5
#define VISION_LOSE_FRAMES             10
#define VISION_FULL_FRAMES             30
#define VISION_SCAN_INTERVAL_MS        200

#endif
