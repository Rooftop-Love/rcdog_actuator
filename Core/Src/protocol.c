/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   USART2 command/response protocol implementation.
  *
  *          Receive path: 1 byte per interrupt -> frame-header search state
  *          machine -> on a complete CRC-valid frame, set frame_ready.
  *          The main loop calls Protocol_Process() which de-duplicates on
  *          cmd_id, dispatches the command and sends the 8-byte response.
  *
  *          All heavy work (CRC verify of the full frame, dispatch, response
  *          transmit) runs in the main loop; the ISR only feeds bytes and
  *          re-arms the next HAL_UART_Receive_IT.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "action.h"
/* USER CODE END Includes */

/* USER CODE BEGIN TD */

/* States of the receive state machine. */
typedef enum
{
  RX_WAIT_HEAD0,    /* waiting for 0xA5                       */
  RX_WAIT_HEAD1,    /* got 0xA5, waiting for 0x00             */
  RX_GATHER_BODY    /* header ok, gathering the remaining 10  */
} RxState_t;

/* USER CODE END TD */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */

/* ---- CRC-16/MODBUS lookup table (poly 0xA001, reflet). ------------------ */
/* Generated from the reference bit-shift loop in MCU_PROTOCOL.md.           */
static const uint16_t crc16_table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* ---- Receive state. ----------------------------------------------------- */
/* Indices 0..1 of rx_buf hold the header; 2..11 hold the body.              */
static volatile RxState_t rx_state;
static volatile uint8_t  rx_buf[PROTO_REQUEST_LEN];  /* header + body */
static volatile uint8_t  rx_body_idx;                /* bytes gathered after header */

/* Single byte reused for HAL_UART_Receive_IT. */
static uint8_t rx_byte;

/* Flag + payload handed from ISR to main loop. */
static volatile uint8_t  frame_ready;
static Proto_Request_t   rx_frame;

/* ---- Last cmd_id seen, for de-duplication. ------------------------------ */
static uint16_t last_cmd_id;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

static uint16_t crc16_modbus(const uint8_t *data, uint32_t len);
static void    send_response(uint16_t cmd_id, uint16_t status);
static void    dispatch(const Proto_Request_t *req);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
  * @brief  Compute CRC-16/MODBUS (poly 0xA001, init 0xFFFF, refin/refout).
  * @param  data bytes to cover
  * @param  len  byte count
  * @retval crc value
  */
static uint16_t crc16_modbus(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  uint32_t i;

  for (i = 0; i < len; i++)
  {
    crc = (uint16_t)((crc >> 8) ^ crc16_table[(uint8_t)(crc ^ data[i])]);
  }
  return crc;
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* ---- Response transmit. ------------------------------------------------ */

/**
  * @brief  Assemble and send the 8-byte response (little-endian, CRC over
  *         the first 6 bytes). Blocks until the bytes leave USART2.
  */
static void send_response(uint16_t cmd_id, uint16_t status)
{
  uint8_t  buf[PROTO_RESPONSE_LEN];
  uint16_t crc;

  buf[0] = PROTO_HEAD0;
  buf[1] = PROTO_HEAD1;
  buf[2] = (uint8_t)(cmd_id & 0xFF);
  buf[3] = (uint8_t)((cmd_id >> 8) & 0xFF);
  buf[4] = (uint8_t)(status & 0xFF);
  buf[5] = (uint8_t)((status >> 8) & 0xFF);

  crc = crc16_modbus(buf, PROTO_RESPONSE_CRC_LEN);
  buf[6] = (uint8_t)(crc & 0xFF);
  buf[7] = (uint8_t)((crc >> 8) & 0xFF);

  (void)HAL_UART_Transmit(&huart2, buf, PROTO_RESPONSE_LEN, 50U);
}

/* ---- Command dispatch. ------------------------------------------------- */

/**
  * @brief  Execute one de-duplicated request and reply with a status.
  */
static void dispatch(const Proto_Request_t *req)
{
  switch (req->cmd)
  {
    case CMD_SUCK:
      Action_Suck();
      send_response(req->cmd_id, STATUS_EXECUTING);
      break;

    case CMD_RELEASE:
      Action_Release();
      send_response(req->cmd_id, STATUS_EXECUTING);
      break;

    case CMD_CANCEL:
      Action_Cancel();
      send_response(req->cmd_id, STATUS_IDLE);
      break;

    case CMD_MOVE:
      Action_Move(req->param1, req->param2);
      send_response(req->cmd_id, STATUS_EXECUTING);
      break;

    case CMD_QUERY:
      /* Reply with current status even while busy. */
      send_response(req->cmd_id, Action_GetStatus());
      break;

    case CMD_PLAY_AUDIO:
      Action_PlayAudio(req->param1);
      send_response(req->cmd_id, STATUS_OK);
      break;

    default:
      send_response(req->cmd_id, STATUS_FAIL_NO_BLOCK);
      break;
  }
}

/* ---- Public API. ------------------------------------------------------- */

void Protocol_Init(void)
{
  Action_Init();

  rx_state     = RX_WAIT_HEAD0;
  rx_body_idx  = 0;
  frame_ready  = 0;
  last_cmd_id  = 0;

  (void)memset((void *)rx_buf, 0, sizeof(rx_buf));

  /* Kick off the first interrupt-driven receive. */
  (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}

void Protocol_OnRxByte(uint8_t byte)
{
  switch (rx_state)
  {
    case RX_WAIT_HEAD0:
      if (byte == PROTO_HEAD0)
      {
        rx_buf[0] = byte;
        rx_state = RX_WAIT_HEAD1;
      }
      break;

    case RX_WAIT_HEAD1:
      if (byte == PROTO_HEAD1)
      {
        rx_buf[1] = byte;
        rx_body_idx = 0;
        rx_state = RX_GATHER_BODY;
      }
      else if (byte == PROTO_HEAD0)
      {
        /* A second 0xA5 re-arms the header search. */
        rx_buf[0] = byte;
        /* stay in RX_WAIT_HEAD1 */
      }
      else
      {
        rx_state = RX_WAIT_HEAD0;
      }
      break;

    case RX_GATHER_BODY:
      /* rx_buf[0..1] = header, rx_buf[2..11] = body. */
      rx_buf[2U + rx_body_idx] = byte;
      rx_body_idx++;

      if (rx_body_idx >= (PROTO_REQUEST_LEN - 2U))
      {
        /* Whole frame collected; hand it to the main loop for CRC + work. */
        frame_ready = 1U;
        rx_state = RX_WAIT_HEAD0;
      }
      break;

    default:
      rx_state = RX_WAIT_HEAD0;
      break;
  }
}

void Protocol_Process(void)
{
  uint16_t recv_crc;
  uint16_t calc_crc;
  const uint8_t *body;
  Proto_Request_t req;

  if (!frame_ready)
  {
    return;
  }
  frame_ready = 0;

  /* CRC covers bytes 0..9 of the frame. */
  calc_crc = crc16_modbus((const uint8_t *)rx_buf, PROTO_REQUEST_CRC_LEN);
  recv_crc = (uint16_t)rx_buf[10] | ((uint16_t)rx_buf[11] << 8);
  if (calc_crc != recv_crc)
  {
    /* Corrupt frame: drop silently, host will retransmit. */
    return;
  }

  /* Decode fields little-endian (no struct-packing assumptions). */
  body       = (const uint8_t *)rx_buf;  /* index from here for clarity */
  req.cmd_id = (uint16_t)body[2] | ((uint16_t)body[3] << 8);
  req.cmd    = (uint16_t)body[4] | ((uint16_t)body[5] << 8);
  req.param1 = (uint16_t)body[6] | ((uint16_t)body[7] << 8);
  req.param2 = (uint16_t)body[8] | ((uint16_t)body[9] << 8);

  /* De-duplicate: a repeated cmd_id only re-reports the current status. */
  if (req.cmd_id == last_cmd_id)
  {
    send_response(req.cmd_id, Action_GetStatus());
    return;
  }
  last_cmd_id = req.cmd_id;

  /* New command: execute and reply. */
  rx_frame = req;
  dispatch(&rx_frame);
}

/**
  * @brief  HAL per-byte RX complete callback.
  * @note   Overrides the weak default in stm32f1xx_hal_uart.c. Must run for
  *         every byte received on huart2.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Protocol_OnRxByte(rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
  }
}

/* USER CODE END 1 */
