#include <assert.h>
#include <stdint.h>
#include <math.h>

#define __GIMBAL_PID_H__
#define __STEPPER_H__
#define __BOARD_H__

#define PID_ERROR_BLEND_START 5.0f
#define PID_ERROR_BLEND_END 60.0f
#define PID_GAIN_SLEW_FACTOR 0.22f
#define PID_KP_LARGE 0.6f
#define PID_KI_LARGE 0.2f
#define PID_KD_LARGE 1.0f
#define PID_KP_SMALL 0.5f
#define PID_KI_SMALL 0.3f
#define PID_KD_SMALL 0.2f
#define PID_INT_SEPARATION_THRESHOLD 100
#define PID_OUTPUT_MAX_X 15.0f
#define PID_OUTPUT_MAX_Y 5.0f
#define PIXEL_TO_ANGLE_FACTOR 0.05f
#define GIMBAL_ADDR_X 1
#define GIMBAL_ADDR_Y 1
#define GIMBAL_RPM_DEFAULT 200
#define GIMBAL_ACC_DEFAULT 50
#define GIMBAL_PULSES_PER_DEG 100.0f
#define GIMBAL_COMPENSATION_FACTOR 1.0f
#define GIMBAL_YAW_DELTA_THRESHOLD 0.1f
#define GIMBAL_PITCH_DELTA_THRESHOLD 0.1f
#define GIMBAL_RATE_FEEDFORWARD_FACTOR 0.0f
#define GIMBAL_FEEDFORWARD_MAX_DEG 3.0f
#define GIMBAL_PIXEL_DEADBAND 2.0f
#define GIMBAL_INTEGRAL_LIMIT_DEG 2.0f
#define GIMBAL_DERIVATIVE_FILTER_ALPHA 1.0f
#define GIMBAL_OUTPUT_SLEW_DEG 15.0f

typedef struct {
    float kp_large, ki_large, kd_large;
    float kp_small, ki_small, kd_small;
    float kp_runtime, ki_runtime, kd_runtime;
    float error, last_error, integral, output, last_output;
    float derivative_filtered;
    float output_max, int_separation_threshold, pixel_to_angle;
    uint8_t motor_addr;
    uint16_t rpm;
    uint8_t acc;
} GimbalPID_t;

typedef struct {
    GimbalPID_t x_axis, y_axis;
    float yaw_delta;
    float pitch_delta;
    float yaw_rate;
    float pitch_rate;
    float imu_dt_s;
    float compensation_factor;
    float rate_feedforward_factor;
    uint8_t compensation_enabled;
} GimbalDualPID_t;

typedef struct {
    float output_x_deg;
    float output_y_deg;
    float yaw_feedforward_deg;
    float pitch_feedforward_deg;
    uint8_t dir_x;
    uint8_t dir_y;
    uint32_t pulses_x;
    uint32_t pulses_y;
} GimbalDebugState_t;

void GimbalDualPID_SetIMUFeedforward(GimbalDualPID_t *dual_pid,
                                     float yaw_delta,
                                     float pitch_delta,
                                     float yaw_rate,
                                     float pitch_rate,
                                     float dt_s);
void GimbalDualPID_UpdateFeedforward(GimbalDualPID_t *dual_pid);

static uint32_t last_x_pulses;
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm,
                         uint8_t acc, uint32_t pulses, uint8_t relative,
                         uint8_t sync)
{
    (void)addr; (void)dir; (void)rpm; (void)acc; (void)relative; (void)sync;
    last_x_pulses = pulses;
}
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm,
                         uint8_t acc, uint32_t pulses, uint8_t relative,
                         uint8_t sync)
{
    (void)addr; (void)dir; (void)rpm; (void)acc; (void)pulses;
    (void)relative; (void)sync;
}

#include "../project_1.0 eye/bsp/gimbal_pid.c"

int main(void)
{
    GimbalDualPID_t pid;
    uint8_t i;
    GimbalDualPID_Init(&pid);
    GimbalDualPID_SetCompensation(&pid, 1.0f, 1U);
    GimbalDualPID_SetIMUDelta(&pid, 1.0f, 0.0f);

    GimbalDualPID_Update(&pid, 0.0f, 0.0f);
    assert(last_x_pulses == 8U);
    assert(pid.yaw_delta == 0.0f);

    for (i = 0U; i < 16U; i++) {
        last_x_pulses = 0U;
        GimbalDualPID_Update(&pid, 0.0f, 0.0f);
    }
    assert(last_x_pulses == 0U);

    pid.x_axis.last_output = 1.25f;
    pid.x_axis.output = 1.25f;
    GimbalDualPID_SetIMUDelta(&pid, 1.0f, 0.0f);
    GimbalDualPID_UpdateFeedforward(&pid);
    assert(last_x_pulses == 8U);
    assert(pid.x_axis.last_output == 1.25f);
    assert(pid.x_axis.output == 1.25f);

    /* A one-pixel residual must not become endless relative-position moves. */
    last_x_pulses = 0U;
    pid.x_axis.integral = 1.0f;
    pid.x_axis.last_output = 0.5f;
    GimbalDualPID_Update(&pid, 1.0f, 0.0f);
    assert(last_x_pulses == 0U);
    assert(pid.x_axis.integral == 0.0f);
    assert(pid.x_axis.last_output == 0.0f);
    return 0;
}
