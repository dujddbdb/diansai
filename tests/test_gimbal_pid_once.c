#include <assert.h>
#include <stdint.h>
#include <math.h>

#define __GIMBAL_PID_H__
#define __STEPPER_H__

#define PID_ERROR_THRESHOLD 50
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
#define GIMBAL_COMPENSATION_FACTOR 1.0f
#define GIMBAL_YAW_DELTA_THRESHOLD 0.1f
#define GIMBAL_PIXEL_DEADBAND 2.0f
#define GIMBAL_INTEGRAL_LIMIT_DEG 2.0f

typedef struct {
    float kp_large, ki_large, kd_large;
    float kp_small, ki_small, kd_small;
    float error, last_error, integral, output, last_output;
    float output_max, int_separation_threshold, pixel_to_angle;
    uint8_t motor_addr;
    uint16_t rpm;
    uint8_t acc;
} GimbalPID_t;

typedef struct {
    GimbalPID_t x_axis, y_axis;
    float yaw_delta;
    float compensation_factor;
    uint8_t compensation_enabled;
} GimbalDualPID_t;

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
    GimbalDualPID_Init(&pid);
    GimbalDualPID_SetCompensation(&pid, 1.0f, 1U);
    GimbalDualPID_SetIMUDelta(&pid, 1.0f, 0.0f);

    GimbalDualPID_Update(&pid, 0.0f, 0.0f);
    assert(last_x_pulses == 100U);
    assert(pid.yaw_delta == 0.0f);

    last_x_pulses = 0U;
    GimbalDualPID_Update(&pid, 0.0f, 0.0f);
    assert(last_x_pulses == 0U);

    /* A one-pixel residual must not become endless relative-position moves. */
    pid.x_axis.integral = 1.0f;
    pid.x_axis.last_output = 0.5f;
    GimbalDualPID_Update(&pid, 1.0f, 0.0f);
    assert(last_x_pulses == 0U);
    assert(pid.x_axis.integral == 0.0f);
    assert(pid.x_axis.last_output == 0.0f);
    return 0;
}
