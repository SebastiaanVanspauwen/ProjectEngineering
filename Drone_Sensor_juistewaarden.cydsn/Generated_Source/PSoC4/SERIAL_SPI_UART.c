/***************************************************************************//**
* \file SERIAL_SPI_UART.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SERIAL_PVT.h"
#include "SERIAL_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
    /* Start index to put data into the software receive buffer.*/
    volatile uint32 SERIAL_rxBufferHead;
    /* Start index to get data from the software receive buffer.*/
    volatile uint32 SERIAL_rxBufferTail;
    /**
    * \addtogroup group_globals
    * \{
    */
    /** Sets when internal software receive buffer overflow
    *  was occurred.
    */
    volatile uint8  SERIAL_rxBufferOverflow;
    /** \} globals */
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
    /* Start index to put data into the software transmit buffer.*/
    volatile uint32 SERIAL_txBufferHead;
    /* Start index to get data from the software transmit buffer.*/
    volatile uint32 SERIAL_txBufferTail;
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

#if(SERIAL_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 SERIAL_rxBufferInternal[SERIAL_INTERNAL_RX_BUFFER_SIZE];
#endif /* (SERIAL_INTERNAL_RX_SW_BUFFER) */

#if(SERIAL_INTERNAL_TX_SW_BUFFER)
    volatile uint8 SERIAL_txBufferInternal[SERIAL_TX_BUFFER_SIZE];
#endif /* (SERIAL_INTERNAL_TX_SW_BUFFER) */


#if(SERIAL_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: SERIAL_SpiUartReadRxData
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer.
    *   - RX software buffer is disabled: Returns data element retrieved from
    *     RX FIFO. Undefined data will be returned if the RX FIFO is empty.
    *   - RX software buffer is enabled: Returns data element from the software
    *     receive buffer. Zero value is returned if the software receive buffer
    *     is empty.
    *
    * \return
    *  Next data element from the receive buffer. 
    *  The amount of data bits to be received depends on RX data bits selection 
    *  (the data bit counting starts from LSB of return value).
    *
    * \globalvars
    *  SERIAL_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  SERIAL_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

    #if (SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

        #if (SERIAL_CHECK_RX_SW_BUFFER)
        {
            if (SERIAL_rxBufferHead != SERIAL_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (SERIAL_rxBufferTail + 1u);

                if (SERIAL_INTERNAL_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data from RX software buffer */
                rxData = SERIAL_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                SERIAL_rxBufferTail = locTail;

                #if (SERIAL_CHECK_UART_RTS_CONTROL_FLOW)
                {
                    /* Check if RX Not Empty is disabled in the interrupt */
                    if (0u == (SERIAL_INTR_RX_MASK_REG & SERIAL_INTR_RX_NOT_EMPTY))
                    {
                        /* Enable RX Not Empty interrupt source to continue
                        * receiving data into software buffer.
                        */
                        SERIAL_INTR_RX_MASK_REG |= SERIAL_INTR_RX_NOT_EMPTY;
                    }
                }
                #endif

            }
        }
        #else
        {
            /* Read data from RX FIFO */
            rxData = SERIAL_RX_FIFO_RD_REG;
        }
        #endif

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartGetRxBufferSize
    ****************************************************************************//**
    *
    *  Returns the number of received data elements in the receive buffer.
    *   - RX software buffer disabled: returns the number of used entries in
    *     RX FIFO.
    *   - RX software buffer enabled: returns the number of elements which were
    *     placed in the receive buffer. This does not include the hardware RX FIFO.
    *
    * \return
    *  Number of received data elements.
    *
    * \globalvars
    *  SERIAL_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  SERIAL_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
    #if (SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
    #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

        #if (SERIAL_CHECK_RX_SW_BUFFER)
        {
            locHead = SERIAL_rxBufferHead;

            if(locHead >= SERIAL_rxBufferTail)
            {
                size = (locHead - SERIAL_rxBufferTail);
            }
            else
            {
                size = (locHead + (SERIAL_INTERNAL_RX_BUFFER_SIZE - SERIAL_rxBufferTail));
            }
        }
        #else
        {
            size = SERIAL_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return (size);
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartClearRxBuffer
    ****************************************************************************//**
    *
    *  Clears the receive buffer and RX FIFO.
    *
    * \globalvars
    *  SERIAL_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  SERIAL_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    void SERIAL_SpiUartClearRxBuffer(void)
    {
        #if (SERIAL_CHECK_RX_SW_BUFFER)
        {
            /* Lock from component interruption */
            SERIAL_DisableInt();

            /* Flush RX software buffer */
            SERIAL_rxBufferHead = SERIAL_rxBufferTail;
            SERIAL_rxBufferOverflow = 0u;

            SERIAL_CLEAR_RX_FIFO;
            SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_ALL);

            #if (SERIAL_CHECK_UART_RTS_CONTROL_FLOW)
            {
                /* Enable RX Not Empty interrupt source to continue receiving
                * data into software buffer.
                */
                SERIAL_INTR_RX_MASK_REG |= SERIAL_INTR_RX_NOT_EMPTY;
            }
            #endif
            
            /* Release lock */
            SERIAL_EnableInt();
        }
        #else
        {
            SERIAL_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (SERIAL_RX_DIRECTION) */


#if(SERIAL_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: SERIAL_SpiUartWriteTxData
    ****************************************************************************//**
    *
    *  Places a data entry into the transmit buffer to be sent at the next available
    *  bus time.
    *  This function is blocking and waits until there is space available to put the
    *  requested data in the transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *   The amount of data bits to be transmitted depends on TX data bits selection 
    *   (the data bit counting starts from LSB of txDataByte).
    *
    * \globalvars
    *  SERIAL_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  SERIAL_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void SERIAL_SpiUartWriteTxData(uint32 txData)
    {
    #if (SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locHead;
    #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

        #if (SERIAL_CHECK_TX_SW_BUFFER)
        {
            /* Put data directly into the TX FIFO */
            if ((SERIAL_txBufferHead == SERIAL_txBufferTail) &&
                (SERIAL_SPI_UART_FIFO_SIZE != SERIAL_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                SERIAL_TX_FIFO_WR_REG = txData;
            }
            /* Put data into TX software buffer */
            else
            {
                /* Head index to put data */
                locHead = (SERIAL_txBufferHead + 1u);

                /* Adjust TX software buffer index */
                if (SERIAL_TX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                /* Wait for space in TX software buffer */
                while (locHead == SERIAL_txBufferTail)
                {
                }

                /* TX software buffer has at least one room */

                /* Clear old status of INTR_TX_NOT_FULL. It sets at the end of transfer when TX FIFO is empty. */
                SERIAL_ClearTxInterruptSource(SERIAL_INTR_TX_NOT_FULL);

                SERIAL_PutWordInTxBuffer(locHead, txData);

                SERIAL_txBufferHead = locHead;

                /* Check if TX Not Full is disabled in interrupt */
                if (0u == (SERIAL_INTR_TX_MASK_REG & SERIAL_INTR_TX_NOT_FULL))
                {
                    /* Enable TX Not Full interrupt source to transmit from software buffer */
                    SERIAL_INTR_TX_MASK_REG |= (uint32) SERIAL_INTR_TX_NOT_FULL;
                }
            }
        }
        #else
        {
            /* Wait until TX FIFO has space to put data element */
            while (SERIAL_SPI_UART_FIFO_SIZE == SERIAL_GET_TX_FIFO_ENTRIES)
            {
            }

            SERIAL_TX_FIFO_WR_REG = txData;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartPutArray
    ****************************************************************************//**
    *
    *  Places an array of data into the transmit buffer to be sent.
    *  This function is blocking and waits until there is a space available to put
    *  all the requested data in the transmit buffer. The array size can be greater
    *  than transmit buffer size.
    *
    * \param wrBuf: pointer to an array of data to be placed in transmit buffer. 
    *  The width of the data to be transmitted depends on TX data width selection 
    *  (the data bit counting starts from LSB for each array element).
    * \param count: number of data elements to be placed in the transmit buffer.
    *
    * \globalvars
    *  SERIAL_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  SERIAL_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void SERIAL_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for (i=0u; i < count; i++)
        {
            SERIAL_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartGetTxBufferSize
    ****************************************************************************//**
    *
    *  Returns the number of elements currently in the transmit buffer.
    *   - TX software buffer is disabled: returns the number of used entries in
    *     TX FIFO.
    *   - TX software buffer is enabled: returns the number of elements currently
    *     used in the transmit buffer. This number does not include used entries in
    *     the TX FIFO. The transmit buffer size is zero until the TX FIFO is
    *     not full.
    *
    * \return
    *  Number of data elements ready to transmit.
    *
    * \globalvars
    *  SERIAL_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  SERIAL_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
    #if (SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */

        #if (SERIAL_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = SERIAL_txBufferTail;

            if (SERIAL_txBufferHead >= locTail)
            {
                size = (SERIAL_txBufferHead - locTail);
            }
            else
            {
                size = (SERIAL_txBufferHead + (SERIAL_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = SERIAL_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return (size);
    }


    /*******************************************************************************
    * Function Name: SERIAL_SpiUartClearTxBuffer
    ****************************************************************************//**
    *
    *  Clears the transmit buffer and TX FIFO.
    *
    * \globalvars
    *  SERIAL_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  SERIAL_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void SERIAL_SpiUartClearTxBuffer(void)
    {
        #if (SERIAL_CHECK_TX_SW_BUFFER)
        {
            /* Lock from component interruption */
            SERIAL_DisableInt();

            /* Flush TX software buffer */
            SERIAL_txBufferHead = SERIAL_txBufferTail;

            SERIAL_INTR_TX_MASK_REG &= (uint32) ~SERIAL_INTR_TX_NOT_FULL;
            SERIAL_CLEAR_TX_FIFO;
            SERIAL_ClearTxInterruptSource(SERIAL_INTR_TX_ALL);

            /* Release lock */
            SERIAL_EnableInt();
        }
        #else
        {
            SERIAL_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (SERIAL_TX_DIRECTION) */


/*******************************************************************************
* Function Name: SERIAL_SpiUartDisableIntRx
****************************************************************************//**
*
*  Disables the RX interrupt sources.
*
*  \return
*   Returns the RX interrupt sources enabled before the function call.
*
*******************************************************************************/
uint32 SERIAL_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = SERIAL_GetRxInterruptMode();

    SERIAL_SetRxInterruptMode(SERIAL_NO_INTR_SOURCES);

    return (intSource);
}


/*******************************************************************************
* Function Name: SERIAL_SpiUartDisableIntTx
****************************************************************************//**
*
*  Disables TX interrupt sources.
*
*  \return
*   Returns TX interrupt sources enabled before function call.
*
*******************************************************************************/
uint32 SERIAL_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = SERIAL_GetTxInterruptMode();

    SERIAL_SetTxInterruptMode(SERIAL_NO_INTR_SOURCES);

    return (intSourceMask);
}


#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: SERIAL_PutWordInRxBuffer
    ****************************************************************************//**
    *
    *  Stores a byte/word into the RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param index:      index to store data byte/word in the RX buffer.
    *  \param rxDataByte: byte/word to store.
    *
    *******************************************************************************/
    void SERIAL_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in buffer */
        if (SERIAL_ONE_BYTE_WIDTH == SERIAL_rxDataBits)
        {
            SERIAL_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            SERIAL_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            SERIAL_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_GetWordFromRxBuffer
    ****************************************************************************//**
    *
    *  Reads byte/word from RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \return
    *   Returns byte/word read from RX buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if (SERIAL_ONE_BYTE_WIDTH == SERIAL_rxDataBits)
        {
            value = SERIAL_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) SERIAL_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)SERIAL_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return (value);
    }


    /*******************************************************************************
    * Function Name: SERIAL_PutWordInTxBuffer
    ****************************************************************************//**
    *
    *  Stores byte/word into the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param idx:        index to store data byte/word in the TX buffer.
    *  \param txDataByte: byte/word to store.
    *
    *******************************************************************************/
    void SERIAL_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in buffer */
        if (SERIAL_ONE_BYTE_WIDTH == SERIAL_txDataBits)
        {
            SERIAL_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            SERIAL_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            SERIAL_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_GetWordFromTxBuffer
    ****************************************************************************//**
    *
    *  Reads byte/word from the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param idx: index to get data byte/word from the TX buffer.
    *
    *  \return
    *   Returns byte/word read from the TX buffer.
    *
    *******************************************************************************/
    uint32 SERIAL_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if (SERIAL_ONE_BYTE_WIDTH == SERIAL_txDataBits)
        {
            value = (uint32) SERIAL_txBuffer[idx];
        }
        else
        {
            value  = (uint32) SERIAL_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) SERIAL_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return (value);
    }

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
