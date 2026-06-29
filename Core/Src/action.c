/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    action.c
  * @brief   Actuator action layer implementation.
  *
  *          Owns the task state (current_status) and the physical outputs.
  *          For now every action is a stub that only sets current_status, so
  *          the protocol layer can be exercised end to end. Wire the real
  *          hardware (GPIO / timers / motor driver / audio) inside the
  *          Action_* bodies and add the status progression here.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "action.h"

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* USER CODE BEGIN PD */

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

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

void Action_Init(void)
{
  audio_playing_id = 0;
  current_status = STATUS_IDLE;
}

void Action_Suck(void)
{
  /* TODO: drive the vacuum pump on. */
  current_status = STATUS_EXECUTING;
}

void Action_Release(void)
{
  /* TODO: drive the vacuum pump off / valve open. */
  current_status = STATUS_EXECUTING;
}

void Action_Cancel(void)
{
  /* TODO: abort the running task immediately. */
  /* Stop audio if playing */
  if (audio_playing_id >= 1 && audio_playing_id <= 8) {
    uint8_t idx = (uint8_t)(audio_playing_id - 1);
    HAL_GPIO_WritePin(sound_pins[idx].port, sound_pins[idx].pin, GPIO_PIN_SET);
    audio_playing_id = 0;
  }
  current_status = STATUS_IDLE;
}

void Action_Move(uint16_t x, uint16_t y)
{
  /* TODO: command the motion to (x, y). */
  (void)x;
  (void)y;
  current_status = STATUS_EXECUTING;
}

void Action_PlayAudio(uint16_t id)
{
  if (id < 1 || id > 8) {
    current_status = STATUS_FAIL_NO_BLOCK;
    return;
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
  current_status = STATUS_EXECUTING;
}

void Action_Tick(void)
{
  if (audio_playing_id == 0) {
    return;
  }

  if ((HAL_GetTick() - audio_start_tick) >= AUDIO_PULSE_MS) {
    uint8_t idx = (uint8_t)(audio_playing_id - 1);
    HAL_GPIO_WritePin(sound_pins[idx].port, sound_pins[idx].pin, GPIO_PIN_SET);
    audio_playing_id = 0;
    current_status = STATUS_OK;
  }
}

uint16_t Action_GetStatus(void)
{
  return current_status;
}

/* USER CODE END 1 */
