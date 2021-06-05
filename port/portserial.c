/*
  * FreeModbus Libary: BARE Port
  * Copyright (C) 2006 Christian Walter <wolti@sil.at>
  *
  * Modified by: LEONG She Teng <leongsheteng@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "main.h"
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
     //static void     prvvUARTTxReadyISR( void );
     //static void     prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
uint8_t inputBuffer[2];
extern volatile uint8_t txComplete;

     void            vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if( xRxEnable )
    {
    	HAL_UART_Receive_IT(&huart3, inputBuffer, 1);
    }
    else
    {
    	HAL_UART_AbortReceive_IT(&huart3);
    }
    if( xTxEnable )
    {
        //prvvUARTTxReadyISR(  );
    	pxMBFrameCBTransmitterEmpty();
    }
    else
    {
    	HAL_UART_AbortTransmit_IT(&huart3);
    }
}

void
vMBPortClose( void )
{
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bInitialized = TRUE;

    ( void )ucPORT;

    huart3.Instance = USART3;
    switch ( ucDataBits )
    {

    case 8:
    	huart3.Init.WordLength = UART_WORDLENGTH_8B;
        break;

    case 9:
    	huart3.Init.WordLength = UART_WORDLENGTH_9B;
        break;

    default:
        bInitialized = FALSE;
    }

    switch ( eParity )
    {
    case MB_PAR_NONE:
    	huart3.Init.Parity = UART_PARITY_NONE;
        break;

    case MB_PAR_ODD:
    	huart3.Init.Parity = UART_PARITY_EVEN;
        break;

    case MB_PAR_EVEN:
    	huart3.Init.Parity = UART_PARITY_ODD;
        break;
    }

    if( bInitialized )
    {
    	huart3.Init.BaudRate = ulBaudRate;
    	huart3.Init.StopBits = UART_STOPBITS_1;
    	huart3.Init.Mode = UART_MODE_TX_RX;
    	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    	if (HAL_UART_Init(&huart3) != HAL_OK)
    	{
    		Error_Handler();
    	}
    }

    return bInitialized;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	txComplete = FALSE;
	HAL_UART_Transmit_IT(&huart3, (uint8_t*)&ucByte, 1);
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Receive Byte */
    *pucByte = inputBuffer[0];

    return TRUE;
}



/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
/*static void
prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}*/

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
/*static void
prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
	   pxMBFrameCBByteReceived(  );
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart == &huart3)
   {
	   txComplete = TRUE;
   }
}
