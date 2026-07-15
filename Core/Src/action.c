/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    action.c
  * @brief   Actuator action layer implementation.
  *
  *          Owns the task state (current_status) and the physical outputs.
  *          Drives the configured GPIO/PWM outputs and advances the non-blocking
  *          suck/release/audio sequences from Action_Tick().
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "action.h"

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "tim.h"
/* USER CODE END Includes */

/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* USER CODE BEGIN PD */

/* Servo pulse width in TIM2 ticks (1 tick = 1 us with PSC=71). */
#define SERVO_CCR_0DEG    500U    /*   0 deg ->  500 us */
#define SERVO_CCR_90DEG   1500U   /*  90 deg -> 1500 us */
#define SERVO_CCR_180DEG  2500U   /* 180 deg -> 2500 us */

/* Pump PWM: TIM3 CH1, PSC=71, Period=99 (10kHz). */
#define PUMP_CCR_MAX  99U   /* full speed */
#define PUMP_CCR_OFF   0U   /* stopped */

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */

/* Current task status. volatile: read by protocol layer, written here and
 * (later) by whatever time base drives the task state machine. */
static volatile uint16_t current_status;

/* Audio playback state */
#define AUDIO_PULSE_MS  1000U

static volatile uint8_t  audio_playing_id;   /* 0 = no playback */
static volatile uint32_t audio_start_tick;

static const struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
} sound_pins[8] = {
  { Sound_1_GPIO_Port, Sound_1_Pin },
  { Sound_2_GPIO_Port, Sound_2_Pin },
  { Sound_3_GPIO_Port, Sound_3_Pin },
  { Sound_4_GPIO_Port, Sound_4_Pin },
  { Sound_5_GPIO_Port, Sound_5_Pin },
  { Sound_6_GPIO_Port, Sound_6_Pin },
  { Sound_7_GPIO_Port, Sound_7_Pin },
  { Sound_8_GPIO_Port, Sound_8_Pin },
};

/* Action state machine for suck/release sequences */
typedef enum {
  ACTION_NONE = 0,
  ACTION_SUCK_SERVO_180,
  ACTION_SUCK_PUMP_ON,
  ACTION_SUCK_WAIT_5S,
    ACTION_RELEASE_SERVO_180,
    ACTION_RELEASE_WAIT_3S,
    ACTION_RELEASE_WAIT_1S,
} ActionState_t;

static volatile ActionState_t action_state;
static volatile uint32_t      action_tick_start;

/* Timing constants */
#define SUCK_WAIT_5S_MS    1000U
#define RELEASE_WAIT_3S   1500U
#define RELEASE_WAIT_1S   500U

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

void Action_Init(void)
{
  audio_playing_id = 0;
  action_state = ACTION_NONE;
  current_status = STATUS_IDLE;

  /* Start servo PWM and default to 0 deg. */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CCR_0DEG);

  /* Start pump PWM, default off. */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PUMP_CCR_OFF);
}

void Action_Suck(void)
{
  /* Start suck sequence: servo 180 deg -> pump on -> 1s -> OK. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CCR_180DEG);
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
  action_state = ACTION_SUCK_SERVO_180;
  action_tick_start = HAL_GetTick();
  current_status = STATUS_EXECUTING;
}

void Action_Release(void)
{
  /* Start release sequence: servo 180 deg -> 3s -> pump off + relay reset -> 1s -> servo 0 deg -> done. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CCR_180DEG);
  action_state = ACTION_RELEASE_SERVO_180;
  action_tick_start = HAL_GetTick();
  current_status = STATUS_EXECUTING;
}

void Action_Cancel(void)
{
  /* Stop audio if playing */
  if (audio_playing_id >= 1 && audio_playing_id <= 8) {
    uint8_t idx = (uint8_t)(audio_playing_id - 1);
    HAL_GPIO_WritePin(sound_pins[idx].port, sound_pins[idx].pin, GPIO_PIN_SET);
    audio_playing_id = 0;
  }

  action_state = ACTION_NONE;
  current_status = STATUS_IDLE;
}

void Action_Move(uint16_t x, uint16_t y)
{
  (void)x;
  (void)y;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CCR_0DEG);
  current_status = STATUS_OK;
}

uint16_t Action_PlayAudio(uint16_t id)
{
  if (id < 1 || id > 8) {
    return STATUS_OK;
  }

  uint8_t idx = (uint8_t)(id - 1);

  /* Stop currently playing audio before starting new one */
  if (audio_playing_id >= 1 && audio_playing_id <= 8) {
    uint8_t old_idx = (uint8_t)(audio_playing_id - 1);
    HAL_GPIO_WritePin(sound_pins[old_idx].port, sound_pins[old_idx].pin, GPIO_PIN_SET);
    if (old_idx == idx) {
      HAL_Delay(20);
    }
  }

  HAL_GPIO_WritePin(sound_pins[idx].port, sound_pins[idx].pin, GPIO_PIN_RESET);

  audio_playing_id = id;
  audio_start_tick = HAL_GetTick();

  return STATUS_OK;
}

void Action_Tick(void)
{
  /* Audio completion check */
  if (audio_playing_id != 0) {
    if ((HAL_GetTick() - audio_start_tick) >= AUDIO_PULSE_MS) {
      uint8_t idx = (uint8_t)(audio_playing_id - 1);
      HAL_GPIO_WritePin(sound_pins[idx].port, sound_pins[idx].pin, GPIO_PIN_SET);
      audio_playing_id = 0;
    }
  }

  /* Action state machine */
  switch (action_state) {
    case ACTION_NONE:
      break;

    case ACTION_SUCK_SERVO_180:
      /* Servo already set to 180 deg in Action_Suck(), now start pump. */
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PUMP_CCR_MAX);
      action_state = ACTION_SUCK_PUMP_ON;
      break;

    case ACTION_SUCK_PUMP_ON:
      action_tick_start = HAL_GetTick();
      action_state = ACTION_SUCK_WAIT_5S;
      break;

    case ACTION_SUCK_WAIT_5S:
      if ((HAL_GetTick() - action_tick_start) >= SUCK_WAIT_5S_MS) {
        action_state = ACTION_NONE;
        current_status = STATUS_OK;
      }
      break;

    case ACTION_RELEASE_SERVO_180:
      /* Servo already set to 180 deg in Action_Release(), start 3s wait. */
      action_tick_start = HAL_GetTick();
      action_state = ACTION_RELEASE_WAIT_3S;
      break;

    case ACTION_RELEASE_WAIT_3S:
      if ((HAL_GetTick() - action_tick_start) >= RELEASE_WAIT_3S) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PUMP_CCR_OFF);
        HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
        action_tick_start = HAL_GetTick();
        action_state = ACTION_RELEASE_WAIT_1S;
      }
      break;

    case ACTION_RELEASE_WAIT_1S:
      if ((HAL_GetTick() - action_tick_start) >= RELEASE_WAIT_1S) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CCR_0DEG);
        action_state = ACTION_NONE;
        current_status = STATUS_OK;
      }
      break;
  }
}

uint16_t Action_GetStatus(void)
{
  return current_status;
}

/* USER CODE END 1 */
