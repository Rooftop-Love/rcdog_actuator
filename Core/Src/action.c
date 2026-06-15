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

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

void Action_Init(void)
{
  /* TODO: init actuator GPIO / timers here once they are configured. */
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
  /* TODO: trigger the audio module to play track id. */
  (void)id;
  current_status = STATUS_OK;
}

void Action_Tick(void)
{
  /* TODO: drive the real task state machine here.
   * Called from the main loop. Poll sensors/timers and advance
   * current_status (EXECUTING -> SUCCESS on completion, or
   * -> FAIL_MOTOR / FAIL_TIMEOUT on fault). */
}

uint16_t Action_GetStatus(void)
{
  return current_status;
}

/* USER CODE END 1 */
