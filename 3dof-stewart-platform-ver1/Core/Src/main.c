/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : .c
  * @brief          :  program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DXL_ID_1  1
#define DXL_ID_3  3
#define DXL_ID_5  5

#define DXL_ADDR_TORQUE_ENABLE   0x18
#define DXL_ADDR_GOAL_POSITION   0x1E
#define DXL_ADDR_MOVING_SPEED    0x20

// Measured neutral angles with enough travel in both directions.
#define ID1_NEUTRAL_DEG  125.10f
#define ID3_NEUTRAL_DEG  125.27f
#define ID5_NEUTRAL_DEG  123.51f

// Maximum platform command angle during normal ball control.
#define MAX_DELTA_DEG            5.0f

// Ignore small position errors near the target.
#define DEADZONE_X       8
#define DEADZONE_Y       8

// Change only the corresponding sign if an image axis is reversed.
#define SIGN_X           1.0f
#define SIGN_Y           1.0f

// Small fixed correction for the measured +X/-Y resting bias.
#define BALANCE_TRIM_X_DEG        0.25f
#define BALANCE_TRIM_Y_DEG       -0.30f

// Per-motor direction signs. Change only after a static direction test.
#define ID1_SIGN         1.0f
#define ID3_SIGN         1.0f
#define ID5_SIGN         1.0f

#define ID1_GAIN         1.20f
#define ID3_GAIN         1.25f
#define ID5_GAIN         1.45f
#define ID1_DOWN_GAIN    1.10f
#define ID3_DOWN_GAIN    0.98f

#define DXL_SPEED_VALUE          260
#define DXL_UPDATE_INTERVAL_MS   30
#define DXL_LOST_TIMEOUT_MS      500
#define DXL_SMOOTH_ALPHA         0.50f
#define DXL_MAX_STEP_DEG         0.60f
#define DXL_KICK_MAX_STEP_DEG    1.20f
#define DXL_CAPTURE_FAST_ALPHA   0.70f
#define DXL_CAPTURE_FAST_STEP_DEG 1.50f
#define DXL_STUCK_MAX_STEP_DEG   2.60f

// Normalize the virtual platform coordinates (radius 300) to about -1 to +1.
#define PD_ERROR_SCALE           300.0f
#define PD_KP_X                  7.2f
#define PD_KP_Y                  7.2f
#define PD_KD_X                  2.7f
#define PD_KD_Y                  2.1f
#define PD_DERIVATIVE_LIMIT      2500.0f
#define PD_D_FILTER_ALPHA        0.55f

#define PD_STATIC_THRESHOLD_X    80
#define PD_STATIC_THRESHOLD_Y    60
#define PD_STATIC_BOOST_X        1.2f
#define PD_STATIC_BOOST_Y        1.5f

#define PD_NEAR_ZONE              150
#define PD_NEAR_KP_SCALE          0.60f
#define PD_NEAR_KD_SCALE          1.40f
#define PD_NEAR_MAX_DELTA_DEG     4.8f
#define PD_FAR_MAX_DELTA_DEG      3.8f
#define PD_FAR_MED_SPEED          250.0f
#define PD_FAR_HIGH_SPEED         400.0f
#define PD_FAR_MED_KP_SCALE       0.80f
#define PD_FAR_HIGH_KP_SCALE      0.60f

#define PD_HOLD_ZONE              65
#define PD_HOLD_EXIT_ZONE         90
#define PD_HOLD_KP_SCALE          0.35f
#define PD_HOLD_KD_SCALE          0.90f
#define PD_HOLD_MAX_DELTA_DEG     2.0f
#define PD_HOLD_SPEED_DEADZONE    10.0f
#define PD_HOLD_DERIVATIVE_LIMIT  350.0f
#define PD_CAPTURE_FAST_ENTER_SPEED 95.0f
#define PD_CAPTURE_FAST_EXIT_SPEED  45.0f
#define PD_ZONE_FAST_BRAKE_SPEED  100.0f
#define PD_ZONE_APPROACH_BRAKE_SPEED 60.0f

#define PD_BRAKE_ZONE             235.0f
#define PD_BRAKE_MIN_SPEED        55.0f
#define PD_EARLY_BRAKE_SPEED      150.0f
#define PD_BRAKE_KP_SCALE         0.28f
#define PD_BRAKE_KD_SCALE         2.45f
#define PD_BRAKE_MAX_DELTA_DEG    5.0f
#define PD_ESCAPE_SPEED           140.0f
#define PD_ESCAPE_KP_SCALE        0.38f
#define PD_ESCAPE_KD_SCALE        1.45f
#define PD_ESCAPE_MAX_DELTA_DEG   4.4f
#define PD_CAPTURE_BRAKE_KP_SCALE 0.12f
#define PD_CAPTURE_BRAKE_KD_SCALE 2.05f
#define PD_CAPTURE_BRAKE_MAX_DEG  4.0f
#define STUCK_TIME_MS             900U
#define STUCK_SPEED_THRESHOLD     45.0f
#define STUCK_MOVE_THRESHOLD      14.0f
// Start stuck detection immediately outside the target zone.
#define STUCK_MIN_ERROR           58.0f
#define STUCK_NEAR_ERROR          85.0f
#define STUCK_FAR_ERROR           150.0f
#define STUCK_KICK_NEAR_DEG       7.6f
#define STUCK_KICK_DEG            8.0f
#define STUCK_KICK_FAR_DEG        8.8f
#define STUCK_MAX_DELTA_DEG       8.8f
#define STUCK_KICK_MS             350U
#define STUCK_COOLDOWN_MS         1000U
#define STUCK_RETURN_MS           220U

#define DEG_TO_RAD              0.01745329252f
#define RAD_TO_DEG              57.295779513f
#define IK_BASE_RADIUS_MM       58.0f
#define IK_PLATFORM_HEIGHT_MM   105.0f
#define IK_PLATFORM_RADIUS_MM   75.0f
#define IK_ARM_LENGTH_MM        83.0f
#define IK_ROD_LENGTH_MM        88.0f
#define IK_OUTPUT_SIGN         -1.0f
#define IK_ID1_ANGLE_DEG       150.0f
#define IK_ID3_ANGLE_DEG       -90.0f
#define IK_ID5_ANGLE_DEG        30.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

float smooth_deg1 = ID1_NEUTRAL_DEG;
float smooth_deg3 = ID3_NEUTRAL_DEG;
float smooth_deg5 = ID5_NEUTRAL_DEG;

uint8_t rx_byte;              // One byte received from UART2.
char rx_buffer[50];           // Buffer for one PC command line.
uint8_t rx_index = 0;         // Current receive-buffer index.

volatile int ball_ex = 0;
volatile int ball_ey = 0;
volatile int ball_valid = 0;

volatile uint8_t new_data_flag = 0;

uint16_t dxl_pos1 = 0;
uint16_t dxl_pos3 = 0;
uint16_t dxl_pos5 = 0;

float target_deg1 = ID1_NEUTRAL_DEG;
float target_deg3 = ID3_NEUTRAL_DEG;
float target_deg5 = ID5_NEUTRAL_DEG;

uint32_t last_dxl_update_time = 0;
uint32_t last_rx_time = 0;
uint32_t last_log_time = 0;

int pd_prev_ex = 0;
int pd_prev_ey = 0;
uint32_t pd_prev_time = 0;
uint8_t pd_has_prev = 0;
uint8_t pd_d_filter_ready = 0;
float pd_filtered_dex_dt = 0.0f;
float pd_filtered_dey_dt = 0.0f;
uint8_t pd_capture_active = 0;
uint8_t pd_capture_fast_motion = 0;

uint8_t stuck_has_ref = 0;
int stuck_ref_ex = 0;
int stuck_ref_ey = 0;
uint32_t stuck_ref_time = 0;
uint32_t stuck_slow_start_time = 0;
uint32_t stuck_kick_until = 0;
uint32_t stuck_return_until = 0;
uint32_t stuck_cooldown_until = 0;
uint8_t stuck_kick_active = 0;

int dbg_cmd_x10 = 0;
int dbg_cmd_y10 = 0;
int dbg_d1_10 = 0;
int dbg_d3_10 = 0;
int dbg_d5_10 = 0;
int dbg_speed = 0;
int dbg_approach_speed = 0;
int dbg_p_x10 = 0;
int dbg_p_y10 = 0;
int dbg_d_x10 = 0;
int dbg_d_y10 = 0;
int dbg_cmd_limit_x10 = 0;
int dbg_p_scale_percent = 100;
uint8_t dbg_control_state = 0;
uint8_t dbg_cmd_saturated = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

uint8_t limit_vector_magnitude(float *x, float *y, float max_magnitude)
{
    float magnitude = sqrtf((*x * *x) + (*y * *y));
    if (magnitude <= max_magnitude || magnitude < 0.0001f)
    {
        return 0;
    }

    float scale = max_magnitude / magnitude;
    *x *= scale;
    *y *= scale;
    return 1;
}

uint16_t deg_to_dxl_pos(float deg)
{
    deg = clamp_float(deg, 0.0f, 300.0f);
    return (uint16_t)((deg / 300.0f) * 1023.0f + 0.5f);
}

float IK_SolveArmAngleDeg(float radial_mm, float z_mm)
{
    float r = sqrtf(radial_mm * radial_mm + z_mm * z_mm);
    if (r < 1.0f) return 0.0f;

    float c = (radial_mm * radial_mm + z_mm * z_mm +
               IK_ARM_LENGTH_MM * IK_ARM_LENGTH_MM -
               IK_ROD_LENGTH_MM * IK_ROD_LENGTH_MM) / (2.0f * IK_ARM_LENGTH_MM);
    float ratio = clamp_float(c / r, -1.0f, 1.0f);
    float alpha = atan2f(z_mm, radial_mm);
    float beta = acosf(ratio);

    return (alpha - beta) * RAD_TO_DEG;
}

float IK_LegDeltaDeg(float leg_angle_deg, float pitch_deg, float roll_deg)
{
    float leg_rad = leg_angle_deg * DEG_TO_RAD;
    float ux = cosf(leg_rad);
    float uy = sinf(leg_rad);

    float px = IK_PLATFORM_RADIUS_MM * ux;
    float py = IK_PLATFORM_RADIUS_MM * uy;

    float roll = roll_deg * DEG_TO_RAD;
    float pitch = pitch_deg * DEG_TO_RAD;
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    float x1 = px;
    float y1 = cr * py;
    float z1 = sr * py;

    float x2 = cp * x1 + sp * z1;
    float y2 = y1;
    float z2 = -sp * x1 + cp * z1;

    float bx = IK_BASE_RADIUS_MM * ux;
    float by = IK_BASE_RADIUS_MM * uy;
    float radial = (x2 - bx) * ux + (y2 - by) * uy;
    float z = IK_PLATFORM_HEIGHT_MM + z2;

    float home_radial = IK_PLATFORM_RADIUS_MM - IK_BASE_RADIUS_MM;
    float home_angle = IK_SolveArmAngleDeg(home_radial, IK_PLATFORM_HEIGHT_MM);
    float target_angle = IK_SolveArmAngleDeg(radial, z);

    return IK_OUTPUT_SIGN * (target_angle - home_angle);
}

void IK_ComputeMotorDeltas(float cmd_x_deg, float cmd_y_deg, float *d1, float *d3, float *d5)
{
    float pitch_deg = -cmd_x_deg;
    float roll_deg = cmd_y_deg;

    *d1 = IK_LegDeltaDeg(IK_ID1_ANGLE_DEG, pitch_deg, roll_deg);
    *d3 = IK_LegDeltaDeg(IK_ID3_ANGLE_DEG, pitch_deg, roll_deg);
    *d5 = IK_LegDeltaDeg(IK_ID5_ANGLE_DEG, pitch_deg, roll_deg);
}

void DXL_SendPacket(uint8_t *packet, uint8_t length)
{
    // PB3 controls the 74HC126 bus direction.
    // HIGH: STM32 transmits to the Dynamixel bus.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_UART_Transmit(&huart1, packet, length, 100);

    HAL_Delay(1);

    // LOW: return the bus to receive mode.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void DXL_WriteByte(uint8_t id, uint8_t address, uint8_t value)
{
    uint8_t packet[8];

    uint8_t length = 4;   // Protocol length field: parameter count + 2.
    uint8_t instruction = 0x03; // WRITE

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = length;
    packet[4] = instruction;
    packet[5] = address;
    packet[6] = value;

    uint8_t sum = id + length + instruction + address + value;
    packet[7] = ~sum;

    DXL_SendPacket(packet, 8);
}

void DXL_WriteWord(uint8_t id, uint8_t address, uint16_t value)
{
    uint8_t packet[9];

    uint8_t length = 5;
    uint8_t instruction = 0x03; // WRITE
    uint8_t value_l = value & 0xFF;
    uint8_t value_h = (value >> 8) & 0xFF;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = length;
    packet[4] = instruction;
    packet[5] = address;
    packet[6] = value_l;
    packet[7] = value_h;

    uint8_t sum = id + length + instruction + address + value_l + value_h;
    packet[8] = ~sum;

    DXL_SendPacket(packet, 9);
}

void DXL_TorqueEnable(uint8_t id)
{
    DXL_WriteByte(id, DXL_ADDR_TORQUE_ENABLE, 1);
}

void DXL_SetSpeed(uint8_t id, uint16_t speed)
{
    // AX-12A moving-speed range is 0 to 1023.
    // DXL_SPEED_VALUE limits the commanded motor speed.
    DXL_WriteWord(id, DXL_ADDR_MOVING_SPEED, speed);
}

void DXL_SetGoalPosition(uint8_t id, uint16_t position)
{
    if (position > 1023) position = 1023;
    DXL_WriteWord(id, DXL_ADDR_GOAL_POSITION, position);
}

void DXL_MoveToNeutral(void)
{
    target_deg1 = ID1_NEUTRAL_DEG;
    target_deg3 = ID3_NEUTRAL_DEG;
    target_deg5 = ID5_NEUTRAL_DEG;

    smooth_deg1 = target_deg1;
    smooth_deg3 = target_deg3;
    smooth_deg5 = target_deg5;

    dxl_pos1 = deg_to_dxl_pos(target_deg1);
    dxl_pos3 = deg_to_dxl_pos(target_deg3);
    dxl_pos5 = deg_to_dxl_pos(target_deg5);

    DXL_SetGoalPosition(DXL_ID_1, dxl_pos1);
    DXL_SetGoalPosition(DXL_ID_3, dxl_pos3);
    DXL_SetGoalPosition(DXL_ID_5, dxl_pos5);

    pd_has_prev = 0;
    pd_d_filter_ready = 0;
    pd_filtered_dex_dt = 0.0f;
    pd_filtered_dey_dt = 0.0f;
    pd_capture_active = 0;
    pd_capture_fast_motion = 0;
    pd_prev_ex = 0;
    pd_prev_ey = 0;
    pd_prev_time = HAL_GetTick();
}

void DXL_ControlFromBall(int ex, int ey, int valid)
{
    float cmd_x = 0.0f;
    float cmd_y = 0.0f;
    uint32_t now = HAL_GetTick();

    if (valid == 1)
    {
        int p_ex = ex;
        int p_ey = ey;
        float dt = (float)DXL_UPDATE_INTERVAL_MS / 1000.0f;
        float dex_dt = 0.0f;
        float dey_dt = 0.0f;

        if (p_ex > -DEADZONE_X && p_ex < DEADZONE_X) p_ex = 0;
        if (p_ey > -DEADZONE_Y && p_ey < DEADZONE_Y) p_ey = 0;

        if (pd_has_prev == 1)
        {
            uint32_t elapsed_ms = now - pd_prev_time;
            if (elapsed_ms > 0)
            {
                dt = (float)elapsed_ms / 1000.0f;
            }

            float raw_dex_dt = ((float)(ex - pd_prev_ex)) / dt;
            float raw_dey_dt = ((float)(ey - pd_prev_ey)) / dt;
            raw_dex_dt = clamp_float(raw_dex_dt, -PD_DERIVATIVE_LIMIT, PD_DERIVATIVE_LIMIT);
            raw_dey_dt = clamp_float(raw_dey_dt, -PD_DERIVATIVE_LIMIT, PD_DERIVATIVE_LIMIT);

            if (pd_d_filter_ready == 0)
            {
                pd_filtered_dex_dt = raw_dex_dt;
                pd_filtered_dey_dt = raw_dey_dt;
                pd_d_filter_ready = 1;
            }
            else
            {
                pd_filtered_dex_dt += PD_D_FILTER_ALPHA *
                                      (raw_dex_dt - pd_filtered_dex_dt);
                pd_filtered_dey_dt += PD_D_FILTER_ALPHA *
                                      (raw_dey_dt - pd_filtered_dey_dt);
            }

            dex_dt = pd_filtered_dex_dt;
            dey_dt = pd_filtered_dey_dt;
        }

        pd_prev_ex = ex;
        pd_prev_ey = ey;
        pd_prev_time = now;
        pd_has_prev = 1;

        float ball_speed = sqrtf(dex_dt * dex_dt + dey_dt * dey_dt);
        float error_radius = sqrtf((float)p_ex * (float)p_ex + (float)p_ey * (float)p_ey);
        float radial_speed = 0.0f;
        if (error_radius > 1.0f)
        {
            radial_speed = ((float)p_ex * dex_dt + (float)p_ey * dey_dt) / error_radius;
        }
        float approach_speed = -radial_speed;
        float kp_scale = 1.0f;
        float kd_scale = 1.0f;
        float cmd_limit = PD_FAR_MAX_DELTA_DEG;
        float far_speed_kp_scale = 1.0f;
        uint8_t near_target = 0;
        dbg_control_state = 0;
        if ((p_ex > -PD_NEAR_ZONE && p_ex < PD_NEAR_ZONE) &&
            (p_ey > -PD_NEAR_ZONE && p_ey < PD_NEAR_ZONE))
        {
            near_target = 1;
            dbg_control_state = 1;
            kp_scale = PD_NEAR_KP_SCALE;
            kd_scale = PD_NEAR_KD_SCALE;
            cmd_limit = PD_NEAR_MAX_DELTA_DEG;
        }

        if ((error_radius <= PD_BRAKE_ZONE &&
             approach_speed >= PD_BRAKE_MIN_SPEED) ||
            approach_speed >= PD_EARLY_BRAKE_SPEED)
        {
            near_target = 1;
            dbg_control_state = 2;
            kp_scale = PD_BRAKE_KP_SCALE;
            kd_scale = PD_BRAKE_KD_SCALE;
            cmd_limit = PD_BRAKE_MAX_DELTA_DEG;
        }
        else if ((ball_speed >= PD_ZONE_FAST_BRAKE_SPEED) &&
                 (approach_speed <= -PD_ESCAPE_SPEED))
        {
            near_target = 1;
            dbg_control_state = 2;
            kp_scale = PD_ESCAPE_KP_SCALE;
            kd_scale = PD_ESCAPE_KD_SCALE;
            cmd_limit = PD_ESCAPE_MAX_DELTA_DEG;
        }

        int error_r2 = p_ex * p_ex + p_ey * p_ey;
        if (pd_capture_active != 0)
        {
            if (error_r2 >= (PD_HOLD_EXIT_ZONE * PD_HOLD_EXIT_ZONE))
            {
                pd_capture_active = 0;
            }
        }
        else if (error_r2 <= (PD_HOLD_ZONE * PD_HOLD_ZONE))
        {
            pd_capture_active = 1;
        }

        if (pd_capture_active != 0)
        {
            if (pd_capture_fast_motion != 0)
            {
                if (ball_speed <= PD_CAPTURE_FAST_EXIT_SPEED)
                {
                    pd_capture_fast_motion = 0;
                }
            }
            else if (ball_speed >= PD_CAPTURE_FAST_ENTER_SPEED)
            {
                pd_capture_fast_motion = 1;
            }

            near_target = 1;
            if (approach_speed >= PD_ZONE_APPROACH_BRAKE_SPEED ||
                ball_speed >= PD_ZONE_FAST_BRAKE_SPEED ||
                pd_capture_fast_motion != 0)
            {
                dbg_control_state = 3;
                kp_scale = PD_CAPTURE_BRAKE_KP_SCALE;
                kd_scale = PD_CAPTURE_BRAKE_KD_SCALE;
                cmd_limit = PD_CAPTURE_BRAKE_MAX_DEG;
            }
            else if (error_r2 <= (PD_HOLD_ZONE * PD_HOLD_ZONE))
            {
                dbg_control_state = 4;
                kp_scale = PD_HOLD_KP_SCALE;
                kd_scale = PD_HOLD_KD_SCALE;
                cmd_limit = PD_HOLD_MAX_DELTA_DEG;
            }
            dex_dt = clamp_float(dex_dt, -PD_HOLD_DERIVATIVE_LIMIT, PD_HOLD_DERIVATIVE_LIMIT);
            dey_dt = clamp_float(dey_dt, -PD_HOLD_DERIVATIVE_LIMIT, PD_HOLD_DERIVATIVE_LIMIT);
            if (ball_speed < PD_HOLD_SPEED_DEADZONE)
            {
                dex_dt = 0.0f;
                dey_dt = 0.0f;
            }
        }
        else
        {
            pd_capture_fast_motion = 0;
        }

        // Reduce only the far-field P drive when the ball already has momentum.
        // D damping keeps the full 3.8-degree far-field command limit.
        if (near_target == 0)
        {
            if (ball_speed >= PD_FAR_HIGH_SPEED)
            {
                far_speed_kp_scale = PD_FAR_HIGH_KP_SCALE;
            }
            else if (ball_speed >= PD_FAR_MED_SPEED)
            {
                far_speed_kp_scale = PD_FAR_MED_KP_SCALE;
            }
        }

        float norm_ex = (float)p_ex / PD_ERROR_SCALE;
        float norm_ey = (float)p_ey / PD_ERROR_SCALE;
        float norm_dex_dt = dex_dt / PD_ERROR_SCALE;
        float norm_dey_dt = dey_dt / PD_ERROR_SCALE;
        float p_term_x = (PD_KP_X * kp_scale * far_speed_kp_scale) * norm_ex;
        float p_term_y = (PD_KP_Y * kp_scale * far_speed_kp_scale) * norm_ey;
        float d_term_x = (PD_KD_X * kd_scale) * norm_dex_dt;
        float d_term_y = (PD_KD_Y * kd_scale) * norm_dey_dt;

        cmd_x = p_term_x + d_term_x;
        cmd_y = p_term_y + d_term_y;
        dbg_p_x10 = (int)(p_term_x * 10.0f);
        dbg_p_y10 = (int)(p_term_y * 10.0f);
        dbg_d_x10 = (int)(d_term_x * 10.0f);
        dbg_d_y10 = (int)(d_term_y * 10.0f);

        if (near_target == 0 && ball_speed < PD_FAR_MED_SPEED)
        {
            if (p_ex > PD_STATIC_THRESHOLD_X) cmd_x += PD_STATIC_BOOST_X;
            else if (p_ex < -PD_STATIC_THRESHOLD_X) cmd_x -= PD_STATIC_BOOST_X;

            if (p_ey > PD_STATIC_THRESHOLD_Y) cmd_y += PD_STATIC_BOOST_Y;
            else if (p_ey < -PD_STATIC_THRESHOLD_Y) cmd_y -= PD_STATIC_BOOST_Y;
        }

        dbg_cmd_saturated = limit_vector_magnitude(&cmd_x, &cmd_y, cmd_limit);
        dbg_speed = (int)ball_speed;
        dbg_approach_speed = (int)approach_speed;
        dbg_cmd_limit_x10 = (int)(cmd_limit * 10.0f);
        dbg_p_scale_percent = (int)(far_speed_kp_scale * 100.0f);

        cmd_x *= SIGN_X;
        cmd_y *= SIGN_Y;

        cmd_x += BALANCE_TRIM_X_DEG;
        cmd_y += BALANCE_TRIM_Y_DEG;
        limit_vector_magnitude(&cmd_x, &cmd_y, MAX_DELTA_DEG);
        dbg_cmd_x10 = (int)(cmd_x * 10.0f);
        dbg_cmd_y10 = (int)(cmd_y * 10.0f);

        float d1 = 0.0f;
        float d3 = 0.0f;
        float d5 = 0.0f;
        IK_ComputeMotorDeltas(cmd_x, cmd_y, &d1, &d3, &d5);

        uint8_t outside_hold_zone = ((float)(ex * ex + ey * ey) > (STUCK_MIN_ERROR * STUCK_MIN_ERROR));
        stuck_kick_active = (now < stuck_kick_until) ? 1 : 0;

        if (outside_hold_zone != 0)
        {
            float ref_move = sqrtf((float)((ex - stuck_ref_ex) * (ex - stuck_ref_ex) +
                                           (ey - stuck_ref_ey) * (ey - stuck_ref_ey)));
            uint8_t slow_outside = (ball_speed <= STUCK_SPEED_THRESHOLD) ? 1 : 0;
            if (stuck_has_ref == 0)
            {
                stuck_has_ref = 1;
                stuck_ref_ex = ex;
                stuck_ref_ey = ey;
                stuck_ref_time = now;
                stuck_slow_start_time = slow_outside ? now : 0;
            }
            else if ((ref_move > STUCK_MOVE_THRESHOLD) &&
                     (slow_outside == 0))
            {
                stuck_ref_ex = ex;
                stuck_ref_ey = ey;
                stuck_ref_time = now;
                stuck_slow_start_time = 0;
                stuck_kick_until = 0;
                stuck_kick_active = 0;
            }
            else if (slow_outside != 0)
            {
                if (stuck_slow_start_time == 0)
                {
                    stuck_slow_start_time = now;
                }
                if (((now - stuck_slow_start_time >= STUCK_TIME_MS) ||
                     (now - stuck_ref_time >= STUCK_TIME_MS)) &&
                    (now >= stuck_cooldown_until))
                {
                    stuck_kick_until = now + STUCK_KICK_MS;
                    stuck_return_until = stuck_kick_until + STUCK_RETURN_MS;
                    stuck_cooldown_until = stuck_return_until + STUCK_COOLDOWN_MS;
                    stuck_kick_active = 1;
                    stuck_ref_ex = ex;
                    stuck_ref_ey = ey;
                    stuck_ref_time = now;
                    stuck_slow_start_time = now;
                }
            }
            else if ((now - stuck_ref_time >= STUCK_TIME_MS) &&
                     (now >= stuck_cooldown_until))
            {
                stuck_kick_until = now + STUCK_KICK_MS;
                stuck_return_until = stuck_kick_until + STUCK_RETURN_MS;
                stuck_cooldown_until = stuck_return_until + STUCK_COOLDOWN_MS;
                stuck_kick_active = 1;
                stuck_ref_ex = ex;
                stuck_ref_ey = ey;
                stuck_ref_time = now;
                stuck_slow_start_time = 0;
            }
        }
        else
        {
            stuck_has_ref = 0;
            stuck_slow_start_time = 0;
            stuck_kick_active = 0;
            stuck_kick_until = 0;
        }

        if (stuck_kick_active != 0)
        {
            float mag = sqrtf((float)(ex * ex + ey * ey));
            if (mag > 1.0f)
            {
                float stuck_kick_deg = STUCK_KICK_DEG;
                if (mag < STUCK_NEAR_ERROR)
                {
                    stuck_kick_deg = STUCK_KICK_NEAR_DEG;
                }
                else if (mag > STUCK_FAR_ERROR)
                {
                    stuck_kick_deg = STUCK_KICK_FAR_DEG;
                }

                float kick_x = stuck_kick_deg * (float)ex / mag;
                float kick_y = stuck_kick_deg * (float)ey / mag;

                // Keep the auxiliary bump in the same roll/pitch -> IK path.
                IK_ComputeMotorDeltas(kick_x, kick_y, &d1, &d3, &d5);
            }
        }
        // Apply the calibrated motor gains to both normal IK and stuck-bump IK.
        d1 = d1 * ID1_GAIN;
        if (d1 > 0.0f)
        {
            d1 = d1 * ID1_DOWN_GAIN;
        }
        d3 = d3 * ID3_GAIN;
        if (d3 > 0.0f)
        {
            d3 = d3 * ID3_DOWN_GAIN;
        }
        d5 = d5 * ID5_GAIN;

        float motor_delta_limit = (stuck_kick_active != 0) ?
                                  STUCK_MAX_DELTA_DEG : MAX_DELTA_DEG;
        d1 = clamp_float(d1, -motor_delta_limit, motor_delta_limit);
        d3 = clamp_float(d3, -motor_delta_limit, motor_delta_limit);
        d5 = clamp_float(d5, -motor_delta_limit, motor_delta_limit);

        dbg_d1_10 = (int)(d1 * 10.0f);
        dbg_d3_10 = (int)(d3 * 10.0f);
        dbg_d5_10 = (int)(d5 * 10.0f);

        target_deg1 = ID1_NEUTRAL_DEG + ID1_SIGN * d1;
        target_deg3 = ID3_NEUTRAL_DEG + ID3_SIGN * d3;
        target_deg5 = ID5_NEUTRAL_DEG + ID5_SIGN * d5;
    }
    else
    {
        pd_has_prev = 0;
        pd_d_filter_ready = 0;
        pd_filtered_dex_dt = 0.0f;
        pd_filtered_dey_dt = 0.0f;
        pd_prev_ex = 0;
        pd_prev_ey = 0;
        pd_prev_time = now;
        pd_capture_active = 0;
        pd_capture_fast_motion = 0;
        stuck_has_ref = 0;
        stuck_kick_active = 0;
        stuck_kick_until = 0;
        stuck_return_until = 0;
        stuck_slow_start_time = 0;

        target_deg1 = ID1_NEUTRAL_DEG;
        target_deg3 = ID3_NEUTRAL_DEG;
        target_deg5 = ID5_NEUTRAL_DEG;
        dbg_cmd_x10 = 0;
        dbg_cmd_y10 = 0;
        dbg_d1_10 = 0;
        dbg_d3_10 = 0;
        dbg_d5_10 = 0;
        dbg_speed = 0;
        dbg_approach_speed = 0;
        dbg_p_x10 = 0;
        dbg_p_y10 = 0;
        dbg_d_x10 = 0;
        dbg_d_y10 = 0;
        dbg_cmd_limit_x10 = 0;
        dbg_p_scale_percent = 100;
        dbg_control_state = 0;
        dbg_cmd_saturated = 0;
    }

    target_deg1 = clamp_float(target_deg1, 0.0f, 300.0f);
    target_deg3 = clamp_float(target_deg3, 0.0f, 300.0f);
    target_deg5 = clamp_float(target_deg5, 0.0f, 300.0f);
    float dxl_smooth_alpha = (pd_capture_fast_motion != 0) ?
                             DXL_CAPTURE_FAST_ALPHA : DXL_SMOOTH_ALPHA;
    float next_deg1 = smooth_deg1 + dxl_smooth_alpha * (target_deg1 - smooth_deg1);
    float next_deg3 = smooth_deg3 + dxl_smooth_alpha * (target_deg3 - smooth_deg3);
    float next_deg5 = smooth_deg5 + dxl_smooth_alpha * (target_deg5 - smooth_deg5);
    float dxl_step_limit;
    if ((stuck_kick_active != 0) || (now < stuck_return_until))
    {
        dxl_step_limit = DXL_STUCK_MAX_STEP_DEG;
    }
    else if (pd_capture_fast_motion != 0)
    {
        dxl_step_limit = DXL_CAPTURE_FAST_STEP_DEG;
    }
    else if (pd_capture_active == 0)
    {
        dxl_step_limit = DXL_KICK_MAX_STEP_DEG;
    }
    else
    {
        dxl_step_limit = DXL_MAX_STEP_DEG;
    }
    smooth_deg1 += clamp_float(next_deg1 - smooth_deg1, -dxl_step_limit, dxl_step_limit);
    smooth_deg3 += clamp_float(next_deg3 - smooth_deg3, -dxl_step_limit, dxl_step_limit);
    smooth_deg5 += clamp_float(next_deg5 - smooth_deg5, -dxl_step_limit, dxl_step_limit);

    dxl_pos1 = deg_to_dxl_pos(smooth_deg1);
    dxl_pos3 = deg_to_dxl_pos(smooth_deg3);
    dxl_pos5 = deg_to_dxl_pos(smooth_deg5);

    DXL_SetGoalPosition(DXL_ID_1, dxl_pos1);
    DXL_SetGoalPosition(DXL_ID_3, dxl_pos3);
    DXL_SetGoalPosition(DXL_ID_5, dxl_pos5);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // A newline terminates one command received from the PC.
        if (rx_byte == '\n')
        {
            rx_buffer[rx_index] = '\0';  // Terminate the received string.

            // Accept "ex,ey,valid".
            int parsed_ex = 0;
            int parsed_ey = 0;
            int parsed_valid = 0;
            int parsed_count = sscanf(rx_buffer, "%d,%d,%d",
                                      &parsed_ex, &parsed_ey, &parsed_valid);

            if (parsed_count == 3)
            {
                if (parsed_valid != 0) parsed_valid = 1;

                ball_ex = parsed_ex;
                ball_ey = parsed_ey;
                ball_valid = parsed_valid;
                new_data_flag = 1;
            }

            // Clear the buffer after processing the complete command.
            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }
        else
        {
            // Store the byte while space remains in the buffer.
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = rx_byte;
            }
            else
            {
                // Discard an oversized command and reset the buffer.
                rx_index = 0;
                memset(rx_buffer, 0, sizeof(rx_buffer));
            }
        }

        // Arm the interrupt for the next UART2 byte.
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start interrupt-driven reception from the PC on USART2.
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  char start_msg[] = "STM UART Receive Start\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), 100);

  // Initialize the Dynamixel motors.
  HAL_Delay(1000);

  DXL_TorqueEnable(DXL_ID_1);
  DXL_TorqueEnable(DXL_ID_3);
  DXL_TorqueEnable(DXL_ID_5);

  HAL_Delay(100);

  DXL_SetSpeed(DXL_ID_1, DXL_SPEED_VALUE);
  DXL_SetSpeed(DXL_ID_3, DXL_SPEED_VALUE);
  DXL_SetSpeed(DXL_ID_5, DXL_SPEED_VALUE);

  HAL_Delay(100);

  // Start at configured neutral angles.
  DXL_MoveToNeutral();
  last_dxl_update_time = HAL_GetTick();
  last_rx_time = HAL_GetTick();

  HAL_Delay(1000);

  char dxl_msg[] = "Dynamixel Init Done\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)dxl_msg, strlen(dxl_msg), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();

      if (new_data_flag == 1)
      {
          int local_ex;
          int local_ey;
          int local_valid;

          __disable_irq();
          local_ex = ball_ex;
          local_ey = ball_ey;
          local_valid = ball_valid;
          new_data_flag = 0;
          __enable_irq();

          last_rx_time = now;

          if (local_valid == 1)
          {
              HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          }

          if (now - last_dxl_update_time >= DXL_UPDATE_INTERVAL_MS)
          {
              DXL_ControlFromBall(local_ex, local_ey, local_valid);
              last_dxl_update_time = now;
          }

          if (now - last_log_time >= 100U)
          {
              char msg[220];
              snprintf(msg, sizeof(msg),
                       "CTRL e=%d,%d st=%u v=%d ap=%d ps=%d P=%d,%d D=%d,%d C=%d,%d lim=%d sat=%u kick=%u M=%d,%d,%d\r\n",
                       local_ex, local_ey, dbg_control_state,
                       dbg_speed, dbg_approach_speed, dbg_p_scale_percent,
                       dbg_p_x10, dbg_p_y10,
                       dbg_d_x10, dbg_d_y10,
                       dbg_cmd_x10, dbg_cmd_y10,
                       dbg_cmd_limit_x10,
                       dbg_cmd_saturated, stuck_kick_active,
                       dbg_d1_10, dbg_d3_10, dbg_d5_10);
              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
              last_log_time = now;
          }
      }
      else if (now - last_rx_time >= DXL_LOST_TIMEOUT_MS &&
               now - last_dxl_update_time >= DXL_UPDATE_INTERVAL_MS)
      {
          DXL_ControlFromBall(0, 0, 0);
          last_dxl_update_time = now;
      }

  }
  /* USER CODE END 3 */
  }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */























































