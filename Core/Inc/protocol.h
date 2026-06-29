/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   USART2 command/response protocol (see MCU_PROTOCOL.md).
  *
  *          Frame layout (little-endian):
  *            Request  (12B): head[2]=0xA5,0x00 | cmd_id[2] | cmd[2]
  *                            | param1[2] | param2[2] | crc16[2]
  *            Response ( 8B): head[2]=0xA5,0x00 | cmd_id[2] | status[2] | crc16[2]
  *
  *          CRC-16/MODBUS: poly 0xA001, init 0xFFFF, refin/refout = true.
  *          Request CRC covers bytes 0..9; response CRC covers bytes 0..5.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* Frame header bytes (sent in this order). */
#define PROTO_HEAD0                 ((uint8_t)0xA5)
#define PROTO_HEAD1                 ((uint8_t)0x00)

/* Total byte counts. */
#define PROTO_REQUEST_LEN           12U
#define PROTO_RESPONSE_LEN          8U

/* CRC coverage (bytes counted from the start of the frame). */
#define PROTO_REQUEST_CRC_LEN       10U
#define PROTO_RESPONSE_CRC_LEN      6U

/* Command codes. */
#define CMD_SUCK                    ((uint16_t)0x0001)
#define CMD_RELEASE                 ((uint16_t)0x0002)
#define CMD_CANCEL                  ((uint16_t)0x0003)
#define CMD_MOVE                    ((uint16_t)0x0100)
#define CMD_QUERY                   ((uint16_t)0x0200)
#define CMD_PLAY_AUDIO              ((uint16_t)0x0300)

/* Status codes. */
#define STATUS_IDLE                 ((uint16_t)0x0000)
#define STATUS_EXECUTING            ((uint16_t)0x0001)
#define STATUS_OK                   ((uint16_t)0x0002)
#define STATUS_FAIL_NO_BLOCK        ((uint16_t)0x0003)
#define STATUS_FAIL_TIMEOUT         ((uint16_t)0x0004)
#define STATUS_FAIL_MOTOR           ((uint16_t)0x0005)

/* USER CODE END PD */

/* USER CODE BEGIN PTD */

/** Decoded request frame, populated once a complete frame passes CRC. */
typedef struct
{
  uint16_t cmd_id;    /* sequence id echoed back, used for de-duplication */
  uint16_t cmd;       /* command code (CMD_*) */
  uint16_t param1;    /* first parameter */
  uint16_t param2;    /* second parameter */
} Proto_Request_t;

/* USER CODE END PTD */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialise the protocol module and start the first RX on both
  *         USART1 (debug) and USART2 (command).
  * @note   Must be called once after MX_USART1_UART_Init() and MX_USART2_UART_Init().
  */
void Protocol_Init(void);

/**
  * @brief  Process a fully-received frame: de-duplicate, dispatch, respond.
  * @note   Call from the main loop; never from an ISR.
  */
void Protocol_Process(void);

/* USER CODE BEGIN EFN */

/* USER CODE END EFN */

#endif /* PROTOCOL_H */
