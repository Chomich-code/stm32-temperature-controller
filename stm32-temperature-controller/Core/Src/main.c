/* USER CODE BEGIN Header */
/*
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 * This software is licensed under terms in the LICENSE file at the
 * root of this software component. If no LICENSE file is provided,
 * this software is provided AS-IS.
 */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

typedef struct
{
    uint8_t raw_state;
    uint8_t stable_state;
    uint32_t last_change_ms;
} ButtonDebounce_t;

typedef enum
{
    MODE_IDLE = 0,
    MODE_HEATING = 1,
    MODE_COOLING = 2
} ThermoMode_t;

/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* Controller settings */

#define SETPOINT_DEFAULT_X2       50
#define SETPOINT_MIN_X2         20
#define SETPOINT_MAX_X2         80
#define SETPOINT_STEP_X2            1
#define BUTTON_DEBOUNCE_MS       40U
#define EMERGENCY_HOLD_MS        1200U

#define HEAT_ON_DELTA_C            1.0f
#define HEAT_OFF_DELTA_C           0.0f
#define COOL_ON_DELTA_C            1.0f
#define COOL_OFF_DELTA_C           0.0f
#define MODE_CHANGE_PAUSE_MS     3000U

#define PID_KP                      15.0f
#define PID_KI                       0.08f
#define PID_KD                       6.0f

#define FAN_USE_PID                  1U
#define FAN_FIXED_PWM_PCT           70U
#define FAN_MIN_ACTIVE_PCT          40U
#define FAN_MAX_PCT                100U

#define FAN_TIM_PRESCALER            0U
#define FAN_TIM_PERIOD             695U

#define FAN_SELFTEST_ENABLED         0U
#define FAN_SELFTEST_MS           3000U
#define FAN_SELFTEST_PCT            70U

#define BME_SAMPLE_PERIOD_MS      1000U
#define BME_TIMEOUT_MS           2500U
#define LCD_REFRESH_MS            500U
#define SENSOR_MIN_VALID_C       -10.0f
#define SENSOR_MAX_VALID_C        85.0f
#define HEATER_MAX_SENSOR_C       35.0f

#if (SETPOINT_DEFAULT_X2 < SETPOINT_MIN_X2) || (SETPOINT_DEFAULT_X2 > SETPOINT_MAX_X2)
#error "Default setpoint is outside the allowed range"
#endif
#if (FAN_MIN_ACTIVE_PCT < 35U) || (FAN_MIN_ACTIVE_PCT > FAN_MAX_PCT) || (FAN_MAX_PCT > 100U)
#error "Invalid fan duty range"
#endif
#if (FAN_FIXED_PWM_PCT < FAN_MIN_ACTIVE_PCT) || (FAN_FIXED_PWM_PCT > FAN_MAX_PCT)
#error "Fixed duty is outside the fan duty range"
#endif
#if (FAN_SELFTEST_PCT < FAN_MIN_ACTIVE_PCT) || (FAN_SELFTEST_PCT > FAN_MAX_PCT)
#error "Self-test duty is outside the fan duty range"
#endif

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

volatile uint8_t bme_id = 0;
volatile HAL_StatusTypeDef bme_status = HAL_ERROR;
volatile HAL_StatusTypeDef temp_status = HAL_ERROR;
volatile float temperature_c = 0.0f;
volatile uint8_t bme_ready = 0;

volatile int16_t setpoint_half_c = SETPOINT_DEFAULT_X2;
volatile uint8_t fan_duty_pct = 0;
volatile uint8_t heater_on = 0;
volatile uint8_t controller_mode = MODE_IDLE;
volatile uint8_t sensor_error = 0;
volatile uint8_t heater_locked = 0;
volatile uint8_t emergency_stop = 0;

static uint16_t bme_address = (0x76U << 1);
static uint16_t dig_T1 = 0;
static int16_t dig_T2 = 0;
static int16_t dig_T3 = 0;
static uint32_t last_read_ms = 0;
static uint32_t last_valid_temp_ms = 0;
static uint32_t last_lcd_ms = 0;
static uint8_t bme_measurement_valid = 0;

static ButtonDebounce_t btn_plus  = {1U, 1U, 0U};
static ButtonDebounce_t btn_minus = {1U, 1U, 0U};
static uint32_t both_pressed_since_ms = 0;
static uint8_t both_hold_latched = 0;

static TIM_HandleTypeDef htim2_pid;
static uint8_t fan_pwm_ready = 0;

static uint8_t fan_selftest_started = 0U;
static uint8_t fan_selftest_done = 0U;
static uint32_t fan_selftest_started_ms = 0U;

static uint32_t last_mode_exit_ms = 0;
static uint8_t mode_exit_valid = 0;

static float pid_integral = 0.0f;
static float pid_rate_filtered = 0.0f;
static float pid_prev_temperature = 0.0f;
static uint8_t pid_prev_valid = 0;
static uint32_t last_pid_ms = 0;
/* USER CODE END PV */

void SystemClock_Config(void);

static void MPU_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define LCD_RS_PORT GPIOA
#define LCD_RS_PIN  GPIO_PIN_0
#define LCD_E_PORT  GPIOE
#define LCD_E_PIN   GPIO_PIN_0
#define LCD_D4_PORT GPIOC
#define LCD_D4_PIN  GPIO_PIN_0
#define LCD_D5_PORT GPIOE
#define LCD_D5_PIN  GPIO_PIN_9
#define LCD_D6_PORT GPIOC
#define LCD_D6_PIN  GPIO_PIN_2
#define LCD_D7_PORT GPIOC
#define LCD_D7_PIN  GPIO_PIN_3

static void LCD_Nibble(uint8_t n)
{
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN,
                      (n & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN,
                      (n & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN,
                      (n & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN,
                      (n & 0x08U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}

static void LCD_Send(uint8_t value, GPIO_PinState rs)
{
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, rs);
    LCD_Nibble((uint8_t)(value >> 4));
    LCD_Nibble((uint8_t)(value & 0x0FU));
}

static void LCD_Command(uint8_t cmd)
{
    LCD_Send(cmd, GPIO_PIN_RESET);
    if ((cmd == 0x01U) || (cmd == 0x02U))
    {
        HAL_Delay(3);
    }
}

static void LCD_Init(void)
{
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);

    LCD_Nibble(0x03U);
    HAL_Delay(5);
    LCD_Nibble(0x03U);
    HAL_Delay(5);
    LCD_Nibble(0x03U);
    HAL_Delay(2);
    LCD_Nibble(0x02U);
    HAL_Delay(2);

    LCD_Command(0x28U);
    LCD_Command(0x08U);
    LCD_Command(0x01U);
    LCD_Command(0x06U);
    LCD_Command(0x0CU);
}

static void LCD_Line(uint8_t row, const char *text)
{
    uint8_t i = 0;
    LCD_Command(row == 0U ? 0x80U : 0xC0U);

    while ((i < 16U) && (text[i] != '\0'))
    {
        LCD_Send((uint8_t)text[i], GPIO_PIN_SET);
        ++i;
    }
    while (i < 16U)
    {
        LCD_Send((uint8_t)' ', GPIO_PIN_SET);
        ++i;
    }
}

/* Return one event per debounced press. */
static uint8_t Button_NewPress(GPIO_TypeDef *port, uint16_t pin,
                               ButtonDebounce_t *button, uint32_t now)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;

    if (raw != button->raw_state)
    {
        button->raw_state = raw;
        button->last_change_ms = now;
    }

    if ((raw != button->stable_state) &&
        ((uint32_t)(now - button->last_change_ms) >= BUTTON_DEBOUNCE_MS))
    {
        button->stable_state = raw;
        if (raw == 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static void Fan_SetDuty(uint8_t percent)
{
    if (percent > FAN_MAX_PCT)
    {
        percent = FAN_MAX_PCT;
    }

    if (percent != 0U)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
        heater_on = 0U;
    }

    uint32_t ticks = (__HAL_TIM_GET_AUTORELOAD(&htim2_pid) + 1U);
    uint32_t compare = (ticks * (uint32_t)percent + 50U) / 100U;
    __HAL_TIM_SET_COMPARE(&htim2_pid, TIM_CHANNEL_3, compare);
    fan_duty_pct = percent;
}

/* TIM2 CH3 is configured here, not by MX_TIM2_Init(). */
static void Fan_PWM_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef pwm = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim2_pid.Instance = TIM2;
    htim2_pid.Init.Prescaler = FAN_TIM_PRESCALER;
    htim2_pid.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2_pid.Init.Period = FAN_TIM_PERIOD;
    htim2_pid.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2_pid.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim2_pid) != HAL_OK)
    {
        Error_Handler();
    }

    pwm.OCMode = TIM_OCMODE_PWM2;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2_pid, &pwm, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    Fan_SetDuty(0U);
    if (HAL_TIM_PWM_Start(&htim2_pid, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    fan_pwm_ready = 1U;
}

static void Heater_Set(uint8_t on)
{

    if (on && fan_pwm_ready)
    {
        Fan_SetDuty(0U);
    }
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    heater_on = on ? 1U : 0U;
}

static float Clamp(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void PID_Reset(void)
{
    pid_integral = 0.0f;
    pid_rate_filtered = 0.0f;
    pid_prev_temperature = 0.0f;
    pid_prev_valid = 0U;
    last_pid_ms = 0U;
}

static void Outputs_Off(void)
{
    Heater_Set(0U);
    if (fan_pwm_ready)
    {
        Fan_SetDuty(0U);
    }
}

static void Mode_Stop(uint32_t now_ms)
{
    Outputs_Off();
    if (controller_mode != MODE_IDLE)
    {
        last_mode_exit_ms = now_ms;
        mode_exit_valid = 1U;
    }
    controller_mode = MODE_IDLE;
    PID_Reset();
}

static void Sensor_Fault(uint32_t now_ms)
{
    sensor_error = 1U;
    Mode_Stop(now_ms);
}

static void Heater_LimitCheck(float temp, uint32_t now_ms)
{
    if (temp >= HEATER_MAX_SENSOR_C)
    {
        heater_locked = 1U;
        Heater_Set(0U);
        if (controller_mode == MODE_HEATING)
        {
            Mode_Stop(now_ms);
        }
    }
}

/* Cooling only; the heater uses on/off control. */
static void Cooling_PID_Update(float temp, float setpoint, uint32_t now_ms)
{

    if (FAN_USE_PID == 0U)
    {
        Fan_SetDuty(FAN_FIXED_PWM_PCT);
        return;
    }

    float dt_s = last_pid_ms == 0U ? 1.0f :
        (float)((uint32_t)(now_ms - last_pid_ms)) / 1000.0f;
    last_pid_ms = now_ms;
    dt_s = Clamp(dt_s, 0.2f, 3.0f);

    float error = temp - setpoint;
    float rate = 0.0f;
    if (pid_prev_valid)
    {
        rate = (temp - pid_prev_temperature) / dt_s;
    }
    pid_prev_temperature = temp;
    pid_prev_valid = 1U;
    pid_rate_filtered = 0.70f * pid_rate_filtered + 0.30f * rate;

    float proposed_i = Clamp(pid_integral + error * dt_s, -300.0f, 300.0f);
    float trial_pwm = (float)FAN_MIN_ACTIVE_PCT + PID_KP * error +
                      PID_KI * proposed_i + PID_KD * pid_rate_filtered;

    if (!((trial_pwm > (float)FAN_MAX_PCT && error > 0.0f) ||
          (trial_pwm < (float)FAN_MIN_ACTIVE_PCT && error < 0.0f)))
    {
        pid_integral = proposed_i;
    }
    float pwm = (float)FAN_MIN_ACTIVE_PCT + PID_KP * error +
                PID_KI * pid_integral + PID_KD * pid_rate_filtered;
    pwm = Clamp(pwm, (float)FAN_MIN_ACTIVE_PCT, (float)FAN_MAX_PCT);
    Fan_SetDuty((uint8_t)(pwm + 0.5f));
}

/* Start at +/-1 C; stop when the setpoint is reached. */
static void Thermostat_Update(uint32_t now_ms, uint8_t new_measurement_or_setpoint)
{
    if (sensor_error || emergency_stop || !bme_ready ||
        !bme_measurement_valid || temp_status != HAL_OK ||
        ((uint32_t)(now_ms - last_valid_temp_ms) > BME_TIMEOUT_MS))
    {
        Mode_Stop(now_ms);
        return;
    }

    float t = temperature_c;
    float target = (float)setpoint_half_c * 0.5f;
    Heater_LimitCheck(t, now_ms);

    if (FAN_SELFTEST_ENABLED && !fan_selftest_done)
    {
        Heater_Set(0U);
        if (!fan_selftest_started)
        {
            fan_selftest_started = 1U;
            fan_selftest_started_ms = now_ms;
            Fan_SetDuty(FAN_SELFTEST_PCT);
            return;
        }
        if ((uint32_t)(now_ms - fan_selftest_started_ms) < FAN_SELFTEST_MS)
        {
            Fan_SetDuty(FAN_SELFTEST_PCT);
            return;
        }
        Fan_SetDuty(0U);
        fan_selftest_done = 1U;
    }

    if ((controller_mode == MODE_COOLING) &&
        (t <= target - COOL_OFF_DELTA_C))
    {
        Mode_Stop(now_ms);
    }

    else if ((controller_mode == MODE_HEATING) &&
             (t >= target + HEAT_OFF_DELTA_C))
    {
        Mode_Stop(now_ms);
    }

    if (controller_mode == MODE_IDLE)
    {
        Outputs_Off();
        if (mode_exit_valid &&
            ((uint32_t)(now_ms - last_mode_exit_ms) < MODE_CHANGE_PAUSE_MS))
        {
            return;
        }

        if (t >= target + COOL_ON_DELTA_C)
        {
            Heater_Set(0U);
            controller_mode = MODE_COOLING;
            PID_Reset();
            Cooling_PID_Update(t, target, now_ms);
            return;
        }
        if (!heater_locked && t <= target - HEAT_ON_DELTA_C)
        {
            Fan_SetDuty(0U);
            controller_mode = MODE_HEATING;
            Heater_Set(1U);
            return;
        }
        return;
    }

    if (controller_mode == MODE_HEATING)
    {
        Fan_SetDuty(0U);

        Heater_Set(1U);
        return;
    }

    if (controller_mode == MODE_COOLING)
    {
        Heater_Set(0U);
        if (new_measurement_or_setpoint)
        {
            Cooling_PID_Update(t, target, now_ms);
        }
    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  MPU_Config();

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();

  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  Heater_Set(0U);
  Fan_PWM_Init();
  Outputs_Off();

  LCD_Init();
  LCD_Line(0, "TEMP CONTROLLER");
  LCD_Line(1, "Starting BME280");
  HAL_Delay(500);

  HAL_Delay(100);

  uint8_t id_temp = 0;

  bme_status = HAL_I2C_Mem_Read(
      &hi2c1,
      bme_address,
      0xD0,
      I2C_MEMADD_SIZE_8BIT,
      &id_temp,
      1,
      100
  );

  if (bme_status != HAL_OK || id_temp != 0x60)
  {

      bme_address = (0x77U << 1);

      bme_status = HAL_I2C_Mem_Read(
          &hi2c1,
          bme_address,
          0xD0,
          I2C_MEMADD_SIZE_8BIT,
          &id_temp,
          1,
          100
      );
  }

  bme_id = id_temp;

  if (bme_status == HAL_OK && bme_id == 0x60)
  {

      uint8_t calib[6];

      bme_status = HAL_I2C_Mem_Read(
          &hi2c1,
          bme_address,
          0x88,
          I2C_MEMADD_SIZE_8BIT,
          calib,
          6,
          100
      );

      if (bme_status == HAL_OK)
      {

          dig_T1 = (uint16_t)(
              calib[0] | (calib[1] << 8)
          );

          dig_T2 = (int16_t)(
              calib[2] | (calib[3] << 8)
          );

          dig_T3 = (int16_t)(
              calib[4] | (calib[5] << 8)
          );

          uint8_t ctrl_meas = 0x23;

          bme_status = HAL_I2C_Mem_Write(
              &hi2c1,
              bme_address,
              0xF4,
              I2C_MEMADD_SIZE_8BIT,
              &ctrl_meas,
              1,
              100
          );

          if (bme_status == HAL_OK)
          {

              HAL_Delay(100);

              bme_ready = 1;
          }
      }
  }

  else
  {

      bme_status = HAL_ERROR;
  }

  if (!bme_ready)
  {
      Sensor_Fault(HAL_GetTick());
      LCD_Line(0, "BME280 NOT FOUND");
      LCD_Line(1, "CHECK I2C");
  }

/* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();
    uint8_t setpoint_changed = 0U;
    uint8_t new_sample = 0U;

    /* Buttons */
    uint8_t plus_press = Button_NewPress(GPIOF, GPIO_PIN_13,
                                        &btn_plus, now_ms);
    uint8_t minus_press = Button_NewPress(GPIOF, GPIO_PIN_14,
                                         &btn_minus, now_ms);
    uint8_t both_down = (btn_plus.stable_state == 0U) &&
                        (btn_minus.stable_state == 0U);

    if (both_down)
    {
        if (both_pressed_since_ms == 0U)
        {
            both_pressed_since_ms = now_ms;
        }
        if (!both_hold_latched &&
            (uint32_t)(now_ms - both_pressed_since_ms) >= EMERGENCY_HOLD_MS)
        {
            both_hold_latched = 1U;
            emergency_stop = 1U;
            Mode_Stop(now_ms);
        }
    }
    else
    {
        both_pressed_since_ms = 0U;
        both_hold_latched = 0U;
    }

    if (!both_down && !emergency_stop)
    {
        if (plus_press && setpoint_half_c < SETPOINT_MAX_X2)
        {
            setpoint_half_c += SETPOINT_STEP_X2;
            if (setpoint_half_c > SETPOINT_MAX_X2)
                setpoint_half_c = SETPOINT_MAX_X2;
            setpoint_changed = 1U;
        }
        if (minus_press && setpoint_half_c > SETPOINT_MIN_X2)
        {
            setpoint_half_c -= SETPOINT_STEP_X2;
            if (setpoint_half_c < SETPOINT_MIN_X2)
                setpoint_half_c = SETPOINT_MIN_X2;
            setpoint_changed = 1U;
        }
    }

    /* Temperature sample */
    if (bme_ready && !sensor_error &&
        (uint32_t)(now_ms - last_read_ms) >= BME_SAMPLE_PERIOD_MS)
    {
        uint8_t raw[3];
        last_read_ms = now_ms;
        temp_status = HAL_I2C_Mem_Read(
            &hi2c1, bme_address, 0xFA, I2C_MEMADD_SIZE_8BIT,
            raw, 3, 100
        );

        if (temp_status == HAL_OK)
        {
            int32_t adc_T = ((int32_t)raw[0] << 12) |
                            ((int32_t)raw[1] << 4) |
                            ((int32_t)raw[2] >> 4);
            if (adc_T != 0x80000)
            {
                float v1 = (((float)adc_T / 16384.0f) -
                            ((float)dig_T1 / 1024.0f)) * (float)dig_T2;
                float diff = ((float)adc_T / 131072.0f) -
                             ((float)dig_T1 / 8192.0f);
                float v2 = diff * diff * (float)dig_T3;
                float t = (v1 + v2) / 5120.0f;

                if (t >= SENSOR_MIN_VALID_C && t < SENSOR_MAX_VALID_C)
                {
                    temperature_c = t;
                    bme_measurement_valid = 1U;
                    last_valid_temp_ms = now_ms;
                    new_sample = 1U;
                }
                else
                {
                    temp_status = HAL_ERROR;
                }
            }
            else
            {
                temp_status = HAL_ERROR;
            }
        }

        if (temp_status != HAL_OK)
        {
            Sensor_Fault(now_ms);
        }
    }

    if (!sensor_error && bme_measurement_valid &&
        (uint32_t)(now_ms - last_valid_temp_ms) > BME_TIMEOUT_MS)
    {
        Sensor_Fault(now_ms);
    }

    Thermostat_Update(now_ms, (uint8_t)(new_sample || setpoint_changed));

    /* Display */
    if (setpoint_changed || new_sample ||
        (uint32_t)(now_ms - last_lcd_ms) >= LCD_REFRESH_MS)
    {
        char line[17];
        last_lcd_ms = now_ms;

        if (!bme_ready || sensor_error || !bme_measurement_valid)
        {
            LCD_Line(0, "BME280 ERROR");
        }
        else
        {
            int32_t t10 = (int32_t)(temperature_c * 10.0f +
                          (temperature_c >= 0.0f ? 0.5f : -0.5f));
            uint32_t ta = (t10 < 0) ? (uint32_t)(-t10) : (uint32_t)t10;
            uint32_t z10 = (uint32_t)setpoint_half_c * 5U;
            uint32_t t_whole = ta / 10U;
            uint32_t z_whole = z10 / 10U;
            uint8_t k = 0U;
            line[k++] = 'T'; line[k++] = ':';
            if (t10 < 0) { line[k++] = '-'; }
            line[k++] = (char)('0' + (t_whole / 10U));
            line[k++] = (char)('0' + (t_whole % 10U));
            line[k++] = '.';
            line[k++] = (char)('0' + (ta % 10U));
            line[k++] = ' '; line[k++] = 'S'; line[k++] = ':';
            line[k++] = (char)('0' + (z_whole / 10U));
            line[k++] = (char)('0' + (z_whole % 10U));
            line[k++] = '.';
            line[k++] = (char)('0' + (z10 % 10U));
            line[k] = '\0';
            LCD_Line(0, line);
        }

        if (emergency_stop)
        {
            LCD_Line(1, "EMERGENCY STOP");
        }
        else if (sensor_error)
        {
            LCD_Line(1, "OUTPUTS DISABLED");
        }
        else if (fan_selftest_started && !fan_selftest_done)
        {
            LCD_Line(1, "FAN TEST 70%");
        }
        else if (controller_mode == MODE_HEATING)
        {
            LCD_Line(1, "HEATING");
        }
        else if (controller_mode == MODE_COOLING)
        {
            (void)snprintf(line, sizeof(line), "COOLING %3u%%",
                           (unsigned)fan_duty_pct);
            LCD_Line(1, line);
        }
        else if (heater_locked)
        {
            LCD_Line(1, "HEATER LIMIT");
        }
        else if (mode_exit_valid &&
                 (uint32_t)(now_ms - last_mode_exit_ms) < MODE_CHANGE_PAUSE_MS)
        {
            LCD_Line(1, "MODE PAUSE");
        }
        else
        {
            LCD_Line(1, "STANDBY");
        }
    }

    HAL_Delay(10);
  }

  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {

    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK

                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {

    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  hi2c1.Instance = I2C1;

  hi2c1.Init.Timing = 0x00303D5B;

  hi2c1.Init.OwnAddress1 = 0;

  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

  hi2c1.Init.OwnAddress2 = 0;

  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;

  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {

    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {

    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {

    Error_Handler();
  }

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOE_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_0, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_0;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void MPU_Config(void)
{

  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;

  MPU_InitStruct.Number = MPU_REGION_NUMBER0;

  MPU_InitStruct.BaseAddress = 0x0;

  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;

  MPU_InitStruct.SubRegionDisable = 0x87;

  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;

  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;

  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{

  /* USER CODE BEGIN Error_Handler_Debug */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
  if (fan_pwm_ready)
  {
      Fan_SetDuty(0U);
  }
  __disable_irq();

  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}

#endif
