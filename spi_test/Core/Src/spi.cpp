/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spi.c
  * @author  MCD Application Team
  * @brief   SPI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      The SPI HAL driver can be used as follows:

      (#) Declare a SPI_HandleTypeDef handle structure, for example:
          SPI_HandleTypeDef  hspi;

      (#)Initialize the SPI low level resources by implementing the HAL_SPI_MspInit() API:
          (##) Enable the SPIx interface clock
          (##) SPI pins configuration
              (+++) Enable the clock for the SPI GPIOs
              (+++) Configure these SPI pins as alternate function push-pull
          (##) NVIC configuration if you need to use interrupt process
              (+++) Configure the SPIx interrupt priority
              (+++) Enable the NVIC SPI IRQ handle
          (##) DMA Configuration if you need to use DMA process
              (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive Stream/Channel
              (+++) Enable the DMAx clock
              (+++) Configure the DMA handle parameters
              (+++) Configure the DMA Tx or Rx Stream/Channel
              (+++) Associate the initialized hdma_tx(or _rx)  handle to the hspi DMA Tx or Rx handle
              (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx or Rx Stream/Channel

      (#) Program the Mode, BidirectionalMode , Data size, Baudrate Prescaler, NSS
          management, Clock polarity and phase, FirstBit and CRC configuration in the hspi Init structure.

      (#) Initialize the SPI registers by calling the HAL_SPI_Init() API:
          (++) This API configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
              by calling the customized HAL_SPI_MspInit() API.
     [..]
       Circular mode restriction:
      (#) The DMA circular mode cannot be used when the SPI is configured in these modes:
          (##) Master 2Lines RxOnly
          (##) Master 1Line Rx
      (#) The CRC feature is not managed when the DMA circular mode is enabled
      (#) When the SPI DMA Pause/Stop features are used, we must use the following APIs
          the HAL_SPI_DMAPause()/ HAL_SPI_DMAStop() only under the SPI callbacks
     [..]
       Master Receive mode restriction:
      (#) In Master unidirectional receive-only mode (MSTR =1, BIDIMODE=0, RXONLY=1) or
          bidirectional receive mode (MSTR=1, BIDIMODE=1, BIDIOE=0), to ensure that the SPI
          does not initiate a new transfer the following procedure has to be respected:
          (##) HAL_SPI_DeInit()
          (##) HAL_SPI_Init()
     [..]
       Callback registration:

      (#) The compilation flag USE_HAL_SPI_REGISTER_CALLBACKS when set to 1U
          allows the user to configure dynamically the driver callbacks.
          Use Functions HAL_SPI_RegisterCallback() to register an interrupt callback.

          Function HAL_SPI_RegisterCallback() allows to register following callbacks:
            (++) TxCpltCallback        : SPI Tx Completed callback
            (++) RxCpltCallback        : SPI Rx Completed callback
            (++) TxRxCpltCallback      : SPI TxRx Completed callback
            (++) TxHalfCpltCallback    : SPI Tx Half Completed callback
            (++) RxHalfCpltCallback    : SPI Rx Half Completed callback
            (++) TxRxHalfCpltCallback  : SPI TxRx Half Completed callback
            (++) ErrorCallback         : SPI Error callback
            (++) AbortCpltCallback     : SPI Abort callback
            (++) MspInitCallback       : SPI Msp Init callback
            (++) MspDeInitCallback     : SPI Msp DeInit callback
          This function takes as parameters the HAL peripheral handle, the Callback ID
          and a pointer to the user callback function.


      (#) Use function HAL_SPI_UnRegisterCallback to reset a callback to the default
          weak function.
          HAL_SPI_UnRegisterCallback takes as parameters the HAL peripheral handle,
          and the Callback ID.
          This function allows to reset following callbacks:
            (++) TxCpltCallback        : SPI Tx Completed callback
            (++) RxCpltCallback        : SPI Rx Completed callback
            (++) TxRxCpltCallback      : SPI TxRx Completed callback
            (++) TxHalfCpltCallback    : SPI Tx Half Completed callback
            (++) RxHalfCpltCallback    : SPI Rx Half Completed callback
            (++) TxRxHalfCpltCallback  : SPI TxRx Half Completed callback
            (++) ErrorCallback         : SPI Error callback
            (++) AbortCpltCallback     : SPI Abort callback
            (++) MspInitCallback       : SPI Msp Init callback
            (++) MspDeInitCallback     : SPI Msp DeInit callback

       [..]
       By default, after the HAL_SPI_Init() and when the state is HAL_SPI_STATE_RESET
       all callbacks are set to the corresponding weak functions:
       examples HAL_SPI_MasterTxCpltCallback(), HAL_SPI_MasterRxCpltCallback().
       Exception done for MspInit and MspDeInit functions that are
       reset to the legacy weak functions in the HAL_SPI_Init()/ HAL_SPI_DeInit() only when
       these callbacks are null (not registered beforehand).
       If MspInit or MspDeInit are not null, the HAL_SPI_Init()/ HAL_SPI_DeInit()
       keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.

       [..]
       Callbacks can be registered/unregistered in HAL_SPI_STATE_READY state only.
       Exception done MspInit/MspDeInit functions that can be registered/unregistered
       in HAL_SPI_STATE_READY or HAL_SPI_STATE_RESET state,
       thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
       Then, the user first registers the MspInit/MspDeInit user callbacks
       using HAL_SPI_RegisterCallback() before calling HAL_SPI_DeInit()
       or HAL_SPI_Init() function.

       [..]
       When the compilation define USE_HAL_PPP_REGISTER_CALLBACKS is set to 0 or
       not defined, the callback registering feature is not available
       and weak (surcharged) callbacks are used.

     [..]
       Using the HAL it is not possible to reach all supported SPI frequency with the different SPI Modes,
       the following table resume the max SPI frequency reached with data size 8bits/16bits,
         according to frequency of the APBx Peripheral Clock (fPCLK) used by the SPI instance.

  @endverbatim

  Additional table :

       DataSize = SPI_DATASIZE_8BIT:
       +----------------------------------------------------------------------------------------------+
       |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
       | Process | Transfer mode  |---------------------|----------------------|----------------------|
       |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
       |==============================================================================================|
       |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    /    |     Interrupt  | Fpclk/4  | Fpclk/8  |    NA     |    NA    |    NA     |   NA     |
       |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    R    |     Interrupt  | Fpclk/8  | Fpclk/8  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    T    |     Interrupt  | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
       +----------------------------------------------------------------------------------------------+

       DataSize = SPI_DATASIZE_16BIT:
       +----------------------------------------------------------------------------------------------+
       |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
       | Process | Transfer mode  |---------------------|----------------------|----------------------|
       |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
       |==============================================================================================|
       |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    /    |     Interrupt  | Fpclk/4  | Fpclk/4  |    NA     |    NA    |    NA     |   NA     |
       |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/32  | Fpclk/2  |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    R    |     Interrupt  | Fpclk/4  | Fpclk/4  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/32 |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    T    |     Interrupt  | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
       +----------------------------------------------------------------------------------------------+
       @note The max SPI frequency depend on SPI data size (8bits, 16bits),
             SPI mode(2 Lines fullduplex, 2 lines RxOnly, 1 line TX/RX) and Process mode (Polling, IT, DMA).
       @note
            (#) TX/RX processes are HAL_SPI_TransmitReceive(), HAL_SPI_TransmitReceive_IT() and HAL_SPI_TransmitReceive_DMA()
            (#) RX processes are HAL_SPI_Receive(), HAL_SPI_Receive_IT() and HAL_SPI_Receive_DMA()
            (#) TX processes are HAL_SPI_Transmit(), HAL_SPI_Transmit_IT() and HAL_SPI_Transmit_DMA()

  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "local.hpp"
#include "spi.hpp"

#include <iostream>
#include <map>
#include <vector>
#include <iomanip>

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup SPI SPI
  * @brief SPI HAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup SPI_Private_Constants SPI Private Constants
  * @{
  */
//#define SPI_DEFAULT_TIMEOUT 100U
//#define SPI_BSY_FLAG_WORKAROUND_TIMEOUT 1000U /*!< Timeout 1000 µs             */
#define SPI_DEFAULT_TIMEOUT 100U
#define SPI_BSY_FLAG_WORKAROUND_TIMEOUT 100000U /*!< Timeout 100 ms             */
/**
  * @}
  */

void printSPIHandle(SPI_HandleTypeDef* spiHandle);

void ResetAndReinitializeSPI(SPI_HandleTypeDef *hspi) {
    // Disable the SPI peripheral
    HAL_SPI_DeInit(hspi); // Replace hspi1 with your SPI handle

    // Reset and re-enable the SPI peripheral's RCC clock configuration
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; // Reset SPI1 peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST; // Release reset
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI1 peripheral clock

    // Reinitialize the SPI peripheral with your desired settings
    // Configure GPIO pins, SPI parameters, etc.
    // For example:
    // SPI_Init();
}



static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  __IO uint32_t count;
  uint32_t tmp_timeout;
  uint32_t tmp_tickstart;

  /* Adjust Timeout value  in case of end of transfer */
  tmp_timeout   = Timeout - (HAL_GetTick() - Tickstart);
  tmp_tickstart = HAL_GetTick();

  /* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
  count = tmp_timeout * ((SystemCoreClock * 32U) >> 20U);

  while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
  {
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout) || (tmp_timeout == 0U))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
           on both master and slave sides in order to resynchronize the master
           and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                                     || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);
        }

        /* Reset CRC Calculation */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SPI_RESET_CRC(hspi);
        }

        hspi->State = HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return HAL_TIMEOUT;
      }
      /* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
      if (count == 0U)
      {
        tmp_timeout = 0U;
      }
      count--;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
  /* Timeout in µs */
  __IO uint32_t count = SPI_BSY_FLAG_WORKAROUND_TIMEOUT * (SystemCoreClock / 24U / 1000000U);
  uprintf("SPI_EndRxTxTransaction waiting for %d * %d =  %d \r\n", SPI_BSY_FLAG_WORKAROUND_TIMEOUT, (SystemCoreClock / 24U / 1000000U), count);
  /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
  if (hspi->Init.Mode == SPI_MODE_MASTER)
  {
    /* Control the BSY flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      return HAL_TIMEOUT;
    }
  }
  else
  {
    /* Wait BSY flag during 1 Byte time transfer in case of Full-Duplex and Tx transfer
    * If Timeout is reached, the transfer is considered as finish.
    * User have to calculate the timeout value to fit with the time of 1 byte transfer.
    * This time is directly link with the SPI clock from Master device.
    */
    do
    {
      if (count == 0U)
      {
        break;
      }
      count--;
    } while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) != RESET);
  }

  uprintf("SPI_EndRxTxTransaction OK SPI_FLAG_BSY=0 end count=%d \r\n", count);
  return HAL_OK;
}


/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup SPI_Private_Functions SPI Private Functions
  * @{
  */
HAL_StatusTypeDef PAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout)
{
  uint16_t             initial_TxXferCount;
  uint32_t             tmp_mode;
  HAL_SPI_StateTypeDef tmp_state;
  uint32_t             tickstart;

  /* Variable used to alternate Rx and Tx during transfer */
  uint32_t             txallowed = 1U;
  HAL_StatusTypeDef    errorcode = HAL_OK;

  /* Check Direction parameter */
  assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

  /* Process Locked */
  __HAL_LOCK(hspi);

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();
  uprintf("PAL_SPI_TS: tick=%d\r\n", tickstart);

  /* Init temporary variables */
  tmp_state           = hspi->State;
  tmp_mode            = hspi->Init.Mode;
  initial_TxXferCount = Size;

  if (!((tmp_state == HAL_SPI_STATE_READY) || \
        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
  {
    uprintf("PAL_SPI_TS: error state not ready:%d\r\n", tmp_state);
    errorcode = HAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    uprintf("PAL_SPI_TS: pTxData = pRxData = NULL:%d\r\n");
    errorcode = HAL_ERROR;
    goto error;
  }

  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
  {
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferCount = Size;
  hspi->RxXferSize  = Size;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferCount = Size;
  hspi->TxXferSize  = Size;

  /*Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

  //printSPIHandle(hspi);
  uprintf("PAL_SPI_TS: TxXferCount =%d buffer: %02X\r\n", hspi->TxXferCount, (*hspi->pTxBuffPtr));
  uprintf("PAL_SPI_TS: TxXferCount =%d buffer: %02X\r\n", Size, (uint8_t *) pTxData);

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(hspi);
  }

  {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
    {
      *((__IO uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint8_t);
      hspi->TxXferCount--;
      uprintf("PAL_SPI_TS: initial load TxXferCount=%d RxXferCount=%d txd=%02X rxd=%02X\r\n", hspi->TxXferCount, hspi->RxXferCount, (*hspi->pTxBuffPtr), (*hspi->pRxBuffPtr));
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
    {
      /* Check TXE flag */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U))
      {
        *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr);
        uint8_t txd = (*hspi->pTxBuffPtr);
        hspi->pTxBuffPtr++;
        hspi->TxXferCount--;
        /* Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;
      uprintf("PAL_SPI_TS: SPI_FLAG_TXE so send txd=%02X and next is rx TxXferCount=%d RxXferCount=%d txd=%02X rxd=%02X\r\n", txd, hspi->TxXferCount, hspi->RxXferCount, (*hspi->pTxBuffPtr), (*hspi->pRxBuffPtr));

      }

      /* Wait until RXNE flag is reset */
      if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U))
      {
        (*(uint8_t *)hspi->pRxBuffPtr) = hspi->Instance->DR;
        uint8_t rxd = (*hspi->pRxBuffPtr);
        hspi->pRxBuffPtr++;
        hspi->RxXferCount--;
        /* Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
      uprintf("PAL_SPI_TS: SPI_FLAG_RXNE so got rxd=%02X and next tx TxXferCount=%d RxXferCount=%d txd=%02X rxd=%02X\r\n", rxd, hspi->TxXferCount, hspi->RxXferCount, (*hspi->pTxBuffPtr), (*hspi->pRxBuffPtr));
      }
      if ((((HAL_GetTick() - tickstart) >=  Timeout) && ((Timeout != HAL_MAX_DELAY))) || (Timeout == 0U))
      {
        errorcode = HAL_TIMEOUT;
        uprintf("PAL_SPI_TS: ERROR TIMEOUT\r\n");
        goto error;
      }
    }
  }


  // TODO: skip check end of transaction
  /* Check the end of the transaction */
/*
  if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK)
  {
    errorcode = HAL_ERROR;
    hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    uprintf("PAL_SPI_TS: ERROR SPI_EndRxTxTransaction not ok\r\n");
    goto error;
  }
  */

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
  }

error :
  hspi->State = HAL_SPI_STATE_READY;
  __HAL_UNLOCK(hspi);
  uprintf("spi error : %d\r\n", errorcode);
  return errorcode;

}



void printSPIStateDescription(HAL_SPI_StateTypeDef state) {
  switch (state) {
    case HAL_SPI_STATE_RESET:
      uprintf("		SPI State: RESET\r\n");
      break;
    case HAL_SPI_STATE_READY:
      uprintf("		SPI State: READY\r\n");
      break;
    case HAL_SPI_STATE_BUSY:
      uprintf("		SPI State: BUSY\r\n");
      break;
    case HAL_SPI_STATE_BUSY_TX:
      uprintf("		SPI State: BUSY_TX\r\n");
      break;
    case HAL_SPI_STATE_BUSY_RX:
      uprintf("		SPI State: BUSY_RX\r\n");
      break;
    case HAL_SPI_STATE_BUSY_TX_RX:
      uprintf("		SPI State: BUSY_TX_RX\r\n");
      break;
    case HAL_SPI_STATE_ERROR:
      uprintf("		SPI State: ERROR\r\n");
      break;
    case HAL_SPI_STATE_ABORT:
      uprintf("		SPI State: ABORT\r\n");
      break;
    default:
      uprintf("		SPI State: unknown\r\n");
      break;
  }
}

// Function to print the values of SPI_TypeDef in hexadecimal format
void printSPIValuesHex(SPI_TypeDef spi) {
  uprintf("	CR1:     0x%08X\r\n", spi.CR1);
  uprintf("	CR2:     0x%08X\r\n", spi.CR2);
  uprintf("	SR:      0x%08X\r\n", spi.SR);
  //uprintf("	DR:      0x%08X\r\n", spi.DR);
  uprintf("	CRCPR:   0x%08X\r\n", spi.CRCPR);
  uprintf("	RXCRCR:  0x%08X\r\n", spi.RXCRCR);
  uprintf("	TXCRCR:  0x%08X\r\n", spi.TXCRCR);
  uprintf("	I2SCFGR: 0x%08X\r\n", spi.I2SCFGR);
  uprintf("	I2SPR:   0x%08X\r\n", spi.I2SPR);
}

// Function to get a string description for HAL_SPI_Error
const char* getSPIErrorDescription(uint32_t errorCode) {
  switch (errorCode) {
    case HAL_SPI_ERROR_NONE:
      return "No error";
    case HAL_SPI_ERROR_MODF:
      return "MODF error";
    case HAL_SPI_ERROR_CRC:
      return "CRC error";
    case HAL_SPI_ERROR_OVR:
      return "OVR error";
    case HAL_SPI_ERROR_FRE:
      return "FRE error";
    case HAL_SPI_ERROR_DMA:
      return "DMA transfer error";
    case HAL_SPI_ERROR_FLAG:
      return "Error on RXNE/TXE/BSY Flag";
    case HAL_SPI_ERROR_ABORT:
      return "Error during SPI Abort procedure";
    default:
      return "Unknown SPI Error";
  }
}

// spi init printing...loads of boilerplate (sigh)

// spi init field names
// see Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_spi.h
// warning: variable definitions in header files can lead to ODR violation...
std::map<std::string, std::vector<std::string>> spiFieldMap = {
      {"Mode", {"SPI_MODE_SLAVE", "SPI_MODE_MASTER"}},
      {"Direction", {"SPI_DIRECTION_2LINES", "SPI_DIRECTION_2LINES_RXONLY", "SPI_DIRECTION_1LINE"}},
      {"DataSize", {"SPI_DATASIZE_8BIT", "SPI_DATASIZE_16BIT"}},
      {"CLKPolarity", {"SPI_POLARITY_LOW", "SPI_POLARITY_HIGH"}},
      {"CLKPhase", {"SPI_PHASE_1EDGE", "SPI_PHASE_2EDGE"}},
      {"NSS", {"SPI_NSS_SOFT", "SPI_NSS_HARD_INPUT", "SPI_NSS_HARD_OUTPUT"}},
      {"BaudRatePrescaler", {"SPI_BAUDRATEPRESCALER_2", "SPI_BAUDRATEPRESCALER_4", "SPI_BAUDRATEPRESCALER_8", "SPI_BAUDRATEPRESCALER_16", "SPI_BAUDRATEPRESCALER_32", "SPI_BAUDRATEPRESCALER_64", "SPI_BAUDRATEPRESCALER_128", "SPI_BAUDRATEPRESCALER_256"}},
      {"FirstBit", {"SPI_FIRSTBIT_MSB", "SPI_FIRSTBIT_LSB"}},
      {"TIMode", {"SPI_TIMODE_DISABLE", "SPI_TIMODE_ENABLE"}},
      {"CRCCalculation", {"SPI_CRCCALCULATION_DISABLE", "SPI_CRCCALCULATION_ENABLE"}
   }
};

std::map<std::string, uint32_t> spiFieldElementMap = {
  {"SPI_MODE_SLAVE",                    SPI_MODE_SLAVE},
  {"SPI_MODE_MASTER",                   SPI_MODE_MASTER},
                                                                      
  {"SPI_DIRECTION_2LINES",              SPI_DIRECTION_2LINES},
  {"SPI_DIRECTION_2LINES_RXONLY",       SPI_DIRECTION_2LINES_RXONLY},
  {"SPI_DIRECTION_1LINE",               SPI_DIRECTION_1LINE},
                                                                      
  {"SPI_DATASIZE_8BIT",                 SPI_DATASIZE_8BIT},
  {"SPI_DATASIZE_16BIT",                SPI_DATASIZE_16BIT},
                                                                      
  {"SPI_POLARITY_LOW",                  SPI_POLARITY_LOW},
  {"SPI_POLARITY_HIGH",                 SPI_POLARITY_HIGH},
                                                                      
  {"SPI_PHASE_1EDGE",                   SPI_PHASE_1EDGE},
  {"SPI_PHASE_2EDGE",                   SPI_PHASE_2EDGE},
                                                                      
  {"SPI_NSS_SOFT",                      SPI_NSS_SOFT},
  {"SPI_NSS_HARD_INPUT",                SPI_NSS_HARD_INPUT},
  {"SPI_NSS_HARD_OUTPUT",               SPI_NSS_HARD_OUTPUT},
                                                                      
  {"SPI_BAUDRATEPRESCALER_2",           SPI_BAUDRATEPRESCALER_2},
  {"SPI_BAUDRATEPRESCALER_4",           SPI_BAUDRATEPRESCALER_4},
  {"SPI_BAUDRATEPRESCALER_8",           SPI_BAUDRATEPRESCALER_8},
  {"SPI_BAUDRATEPRESCALER_16",          SPI_BAUDRATEPRESCALER_16},
  {"SPI_BAUDRATEPRESCALER_32",          SPI_BAUDRATEPRESCALER_32},
  {"SPI_BAUDRATEPRESCALER_64",          SPI_BAUDRATEPRESCALER_64},
  {"SPI_BAUDRATEPRESCALER_128",         SPI_BAUDRATEPRESCALER_128},
  {"SPI_BAUDRATEPRESCALER_256",         SPI_BAUDRATEPRESCALER_256},
                                                                      
                                                                      
  {"SPI_FIRSTBIT_MSB",                  SPI_FIRSTBIT_MSB},
  {"SPI_FIRSTBIT_LSB",                  SPI_FIRSTBIT_LSB},
                                                                      
  {"SPI_TIMODE_DISABLE",                SPI_TIMODE_DISABLE},
  {"SPI_TIMODE_ENABLE",                 SPI_TIMODE_ENABLE},
                                                                      
  {"SPI_CRCCALCULATION_DISABLE",        SPI_CRCCALCULATION_DISABLE},
  {"SPI_CRCCALCULATION_ENABLE",         SPI_CRCCALCULATION_ENABLE},
};


void printInitHelper(uint8_t fieldValue, std::string fieldName) {

  for (auto pound_defines : spiFieldMap[fieldName]) {
    if (spiFieldElementMap[pound_defines] == fieldValue) {
      std::cout << "  " << std::setw(20) << std::left << fieldName << " : " << pound_defines << "\n";
      return;
    }
  }

  std::cout << "  " << std::setw(20) << std::left << fieldName << " : " << "unknown" << "\n";

}

void printSPIInitTypeDef(SPI_InitTypeDef &spiInit) {
  std::cout << " spiHandle->Init: \n";
  printInitHelper(spiInit.Mode,             "Mode");
  printInitHelper(spiInit.Direction,        "Direction");
  printInitHelper(spiInit.DataSize,         "DataSize");
  printInitHelper(spiInit.CLKPolarity,      "CLKPolarity");
  printInitHelper(spiInit.CLKPhase,         "CLKPhase");
  printInitHelper(spiInit.NSS,              "NSS");
  printInitHelper(spiInit.BaudRatePrescaler,"BaudRatePrescaler");
  printInitHelper(spiInit.FirstBit,         "FirstBit");
  printInitHelper(spiInit.TIMode,           "TIMode");
  printInitHelper(spiInit.CRCCalculation,   "CRCCalculation");
  std::cout << "  " << std::setw(20) << std::left << "CRCPolynomial" << " : " << spiInit.CRCPolynomial << "\n";
}

// Function to print the values of SPI_HandleTypeDef including nested structures
void printSPIHandle(SPI_HandleTypeDef* spiHandle) {
  uprintf(" SPI Handle:\r\n");
  uprintf(" Instance: %p\r\n", (void*)spiHandle->Instance);
  printSPIValuesHex(* (spiHandle->Instance));

  uprintf(" SPI_InitTypeDef:\r\n");
  printSPIInitTypeDef(spiHandle->Init);

  uprintf(" pTxBuffPtr: %p ", (void*)spiHandle->pTxBuffPtr);
  uprintf(" TxXferSize: %u ", spiHandle->TxXferSize);
  uprintf(" TxXferCount: %u\r\n", spiHandle->TxXferCount);

  uprintf(" pRxBuffPtr: %p ", (void*)spiHandle->pRxBuffPtr);
  uprintf(" RxXferSize: %u ", spiHandle->RxXferSize);
  uprintf(" RxXferCount: %u\r\n", spiHandle->RxXferCount);

  uprintf(" State: \r\n"); 
  printSPIStateDescription(spiHandle->State);
  uprintf("ErrorCode: 0x%08X : %s\r\n", spiHandle->ErrorCode,getSPIErrorDescription(spiHandle->ErrorCode));
}

