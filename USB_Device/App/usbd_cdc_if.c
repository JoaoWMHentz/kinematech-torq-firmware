/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v3.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
#include <stdbool.h>
#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
static volatile bool cdc_tx_ready = true;

static uint8_t cdc_rx_ring[APP_RX_DATA_SIZE];
static volatile uint32_t cdc_rx_head = 0U;
static volatile uint32_t cdc_rx_tail = 0U;
static volatile bool cdc_rx_overflow = false;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  cdc_tx_ready = true;
  cdc_rx_head = 0U;
  cdc_rx_tail = 0U;
  cdc_rx_overflow = false;
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  const uint32_t length = (Len != NULL) ? *Len : 0U;

  for (uint32_t i = 0U; i < length; ++i)
  {
    const uint8_t byte = Buf[i];
    const uint32_t next_head = (cdc_rx_head + 1U) % APP_RX_DATA_SIZE;

    if (next_head == cdc_rx_tail)
    {
      /* Buffer full: drop newest byte and flag overflow. */
      cdc_rx_overflow = true;
      break;
    }

    cdc_rx_ring[cdc_rx_head] = byte;
    cdc_rx_head = next_head;
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Buf);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  cdc_tx_ready = true;
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static bool USB_DeviceConfigured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

static uint8_t CDC_Transmit_Blocking(const uint8_t *Buf, uint16_t Len, uint32_t timeout_ms)
{
  if ((Buf == NULL) || (Len == 0U))
  {
    return USBD_OK;
  }

  if (!USB_DeviceConfigured())
  {
    /* Device not yet enumerated: pretend success to avoid stalling printf. */
    return USBD_OK;
  }

  uint32_t start_tick = HAL_GetTick();

  /* Wait for any previous transfer to complete. */
  while (!cdc_tx_ready)
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return USBD_BUSY;
    }
    HAL_Delay(1U);
  }

  if (Len > APP_TX_DATA_SIZE)
  {
    Len = APP_TX_DATA_SIZE;
  }

  cdc_tx_ready = false;
  memcpy(UserTxBufferFS, Buf, Len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  const uint8_t result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  if (result != USBD_OK)
  {
    cdc_tx_ready = true;
    return result;
  }

  /* Wait for the transfer complete callback. */
  while (!cdc_tx_ready)
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      cdc_tx_ready = true;
      return USBD_BUSY;
    }
    HAL_Delay(1U);
  }

  return USBD_OK;
}

static bool CDC_FlushTxChunk(uint8_t *chunk, uint32_t *chunk_len, uint32_t timeout_ms)
{
  if ((chunk == NULL) || (chunk_len == NULL))
  {
    return false;
  }

  if (*chunk_len == 0U)
  {
    return true;
  }

  const uint16_t len = (uint16_t)(*chunk_len);
  const uint8_t result = CDC_Transmit_Blocking(chunk, len, timeout_ms);
  if (result != USBD_OK)
  {
    return false;
  }

  *chunk_len = 0U;
  return true;
}

uint32_t CDC_BytesAvailable_FS(void)
{
  uint32_t available;

  __disable_irq();
  if (cdc_rx_head >= cdc_rx_tail)
  {
    available = cdc_rx_head - cdc_rx_tail;
  }
  else
  {
    available = (APP_RX_DATA_SIZE - cdc_rx_tail) + cdc_rx_head;
  }
  __enable_irq();

  return available;
}

uint32_t CDC_Read_FS(uint8_t *Buf, uint32_t Len)
{
  if ((Buf == NULL) || (Len == 0U))
  {
    return 0U;
  }

  uint32_t count = 0U;

  __disable_irq();
  while ((cdc_rx_tail != cdc_rx_head) && (count < Len))
  {
    Buf[count++] = cdc_rx_ring[cdc_rx_tail];
    cdc_rx_tail = (cdc_rx_tail + 1U) % APP_RX_DATA_SIZE;
  }
  __enable_irq();

  return count;
}

bool CDC_RxOverflowed_FS(void)
{
  return cdc_rx_overflow;
}

void CDC_ClearRxOverflow_FS(void)
{
  cdc_rx_overflow = false;
}

int _write(int file, char *ptr, int len)
{
  (void)file;

  if ((ptr == NULL) || (len <= 0))
  {
    return 0;
  }

  uint8_t chunk[APP_TX_DATA_SIZE];
  uint32_t chunk_len = 0U;
  int processed = 0;

  for (int i = 0; i < len; ++i)
  {
    const char ch = ptr[i];

    if (ch == '\n')
    {
      if (chunk_len >= (APP_TX_DATA_SIZE - 2U))
      {
        if (!CDC_FlushTxChunk(chunk, &chunk_len, 100U))
        {
          return processed;
        }
      }
      chunk[chunk_len++] = '\r';
      chunk[chunk_len++] = '\n';
    }
    else
    {
      if (chunk_len >= (APP_TX_DATA_SIZE - 1U))
      {
        if (!CDC_FlushTxChunk(chunk, &chunk_len, 100U))
        {
          return processed;
        }
      }
      chunk[chunk_len++] = (uint8_t)ch;
    }

    ++processed;
  }

  if (!CDC_FlushTxChunk(chunk, &chunk_len, 100U))
  {
    return processed;
  }

  return len;
}

int __io_putchar(int ch)
{
  char tmp = (char)ch;
  (void)_write(0, &tmp, 1);
  return ch;
}

int __io_getchar(void)
{
  uint8_t byte = 0U;

  while (CDC_Read_FS(&byte, 1U) == 0U)
  {
    /* Yield to USB ISR while waiting for data. */
    HAL_Delay(1U);
  }

  return (int)byte;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
