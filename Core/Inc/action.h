/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    action.h
  * @brief   Actuator action layer.
  *
  *          This module owns the device-level task state machine and the
  *          physical outputs (vacuum pump, valve, motion, audio, ...). The
  *          protocol layer (protocol.c) calls these functions to start/cancel
  *          tasks and reads the current status via Action_GetStatus().
  *
  *          Status codes shared with the protocol are defined in protocol.h.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ACTION_H
#define ACTION_H

#include <stdint.h>

/* USER CODE BEGIN Includes */
#include "protocol.h"   /* STATUS_* codes */
/* USER CODE END Includes */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialise the action layer (reset task state).
  * @note   Called by Protocol_Init(); will also drive future hardware init.
  */
void Action_Init(void);

/**
  * @brief  Start a suck (vacuum on) task.
  */
void Action_Suck(void);

/**
  * @brief  Start a release (vacuum off / valve open) task.
  */
void Action_Release(void);

/**
  * @brief  Abort the running task immediately.
  */
void Action_Cancel(void);

/**
  * @brief  Command a move to absolute coordinates.
  * @param  x target x coordinate
  * @param  y target y coordinate
  */
void Action_Move(uint16_t x, uint16_t y);

/**
  * @brief  Trigger playback of an audio track.
  * @param  id audio track id
  */
void Action_PlayAudio(uint16_t id);

/**
  * @brief  Read the current task status.
  * @retval one of the STATUS_* codes from protocol.h
  */
uint16_t Action_GetStatus(void);

/**
  * @brief  Advance the task state machine by one tick.
  * @note   Call from the main loop. Fill in real completion logic here
  *         (e.g. EXECUTING -> SUCCESS once the motor/pump reports done).
  */
void Action_Tick(void);

/* USER CODE BEGIN EFN */

/* USER CODE END EFN */

#endif /* ACTION_H */
