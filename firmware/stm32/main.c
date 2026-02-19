/**
 * main.c — STM32 Motor & Sensor Bridge Firmware
 * -----------------------------------------------
 * Target: STM32F4 (e.g. STM32F411 Nucleo)
 * HAL: STM32 HAL (generated via CubeMX)
 *
 * Responsibilities:
 *   1. Receive 5-byte PWM frames from ROS over UART2 (PA2/PA3)
 *   2. Drive left/right DC motors via TIM3 PWM channels + GPIO direction pins
 *   3. Read 3× HC-SR04 ultrasonic sensors via GPIO trigger/echo
 *   4. Transmit ultrasonic range data back to ROS over UART2
 *
 * Frame format (received from ROS motor_controller.py):
 *   [0xAA][left_pwm][right_pwm][checksum][0x55]
 *
 * Ultrasonic TX frame (sent to ROS):
 *   [0xBB][sensor_id][range_hi][range_lo][0x55]
 *   range = uint16 millimetres
 */

#include "main.h"      // CubeMX generated
#include <string.h>
#include <stdbool.h>

/* ── Constants ────────────────────────────────────────────────────────────── */
#define FRAME_START      0xAA
#define FRAME_END        0x55
#define FRAME_SIZE       5

#define US_FRAME_START   0xBB
#define US_NUM_SENSORS   3
#define SOUND_SPEED_MM_US 0.343f   // mm per microsecond

#define CMD_TIMEOUT_MS   500       // Halt motors if no command within 500 ms

/* ── HAL handles (declared in CubeMX generated main.c / .h) ─────────────── */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;   // Microsecond timer for ultrasonic

/* ── Ultrasonic GPIO config ───────────────────────────────────────────────── */
typedef struct {
    GPIO_TypeDef* trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t      echo_pin;
} USSensor;

static const USSensor us_sensors[US_NUM_SENSORS] = {
    {GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1},   // Front-left
    {GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3},   // Front-right
    {GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5},   // Rear
};

/* ── State ────────────────────────────────────────────────────────────────── */
static uint8_t  rx_buf[FRAME_SIZE];
static uint8_t  rx_byte;
static uint8_t  rx_idx        = 0;
static uint32_t last_cmd_tick = 0;
static bool     estop_active  = false;

/* ── Forward declarations ─────────────────────────────────────────────────── */
static void     apply_motor_left(uint8_t pwm_byte);
static void     apply_motor_right(uint8_t pwm_byte);
static void     halt_motors(void);
static uint32_t us_measure_mm(uint8_t sensor_id);
static void     us_send_frame(uint8_t sensor_id, uint16_t range_mm);
static void     delay_us(uint32_t us);

/* ─────────────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* CubeMX init */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_UART2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // Left motor
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Right motor
    HAL_TIM_Base_Start(&htim4);                  // Microsecond timer

    halt_motors();

    /* Start UART byte-by-byte receive */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    while (1)
    {
        /* Watchdog — halt motors if no valid command within CMD_TIMEOUT_MS */
        if ((HAL_GetTick() - last_cmd_tick) > CMD_TIMEOUT_MS) {
            halt_motors();
        }

        /* Ultrasonic polling — measure each sensor and transmit */
        for (uint8_t i = 0; i < US_NUM_SENSORS; i++) {
            uint32_t range_mm = us_measure_mm(i);
            if (range_mm < 5000) {   // Ignore out-of-range readings
                us_send_frame(i, (uint16_t)range_mm);
            }
            HAL_Delay(10);
        }
    }
}

/* ── UART receive callback (called from ISR) ─────────────────────────────── */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    if (rx_byte == FRAME_START) {
        rx_idx = 0;
    }

    if (rx_idx < FRAME_SIZE) {
        rx_buf[rx_idx++] = rx_byte;
    }

    if (rx_idx == FRAME_SIZE) {
        /* Validate frame */
        if (rx_buf[0] == FRAME_START && rx_buf[4] == FRAME_END) {
            uint8_t left_pwm  = rx_buf[1];
            uint8_t right_pwm = rx_buf[2];
            uint8_t checksum  = rx_buf[3];

            if (((left_pwm + right_pwm) & 0xFF) == checksum) {
                if (!estop_active) {
                    apply_motor_left(left_pwm);
                    apply_motor_right(right_pwm);
                    last_cmd_tick = HAL_GetTick();
                }
            }
        }
        rx_idx = 0;
    }

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* ── Motor drive helpers ──────────────────────────────────────────────────── */
/* pwm_byte: 127=stop, 0=full_reverse, 255=full_forward */

static void apply_motor_left(uint8_t pwm_byte)
{
    int speed = (int)pwm_byte - 127;
    uint32_t duty = (uint32_t)(speed > 0 ? speed : -speed) * 2;
    if (duty > 255) duty = 255;

    /* Direction: PA8=IN1, PA9=IN2 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, speed >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, speed <  0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
}

static void apply_motor_right(uint8_t pwm_byte)
{
    int speed = (int)pwm_byte - 127;
    uint32_t duty = (uint32_t)(speed > 0 ? speed : -speed) * 2;
    if (duty > 255) duty = 255;

    /* Direction: PB0=IN3, PB1=IN4 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, speed >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, speed <  0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}

static void halt_motors(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

/* ── Ultrasonic measurement ───────────────────────────────────────────────── */
static uint32_t us_measure_mm(uint8_t sensor_id)
{
    const USSensor* s = &us_sensors[sensor_id];

    /* 10 µs trigger pulse */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    /* Wait for echo rising edge (timeout 25 ms) */
    uint32_t t_start = __HAL_TIM_GET_COUNTER(&htim4);
    uint32_t deadline = HAL_GetTick() + 25;
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() > deadline) return 9999;
    }
    uint32_t echo_start = __HAL_TIM_GET_COUNTER(&htim4);

    /* Wait for echo falling edge */
    deadline = HAL_GetTick() + 25;
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() > deadline) return 9999;
    }
    uint32_t echo_end = __HAL_TIM_GET_COUNTER(&htim4);

    uint32_t duration_us = echo_end - echo_start;
    return (uint32_t)(duration_us * SOUND_SPEED_MM_US / 2.0f);
}

static void us_send_frame(uint8_t sensor_id, uint16_t range_mm)
{
    uint8_t tx[5];
    tx[0] = US_FRAME_START;
    tx[1] = sensor_id;
    tx[2] = (range_mm >> 8) & 0xFF;
    tx[3] = range_mm & 0xFF;
    tx[4] = FRAME_END;
    HAL_UART_Transmit(&huart2, tx, 5, 10);
}

static void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim4);
    while ((__HAL_TIM_GET_COUNTER(&htim4) - start) < us);
}
