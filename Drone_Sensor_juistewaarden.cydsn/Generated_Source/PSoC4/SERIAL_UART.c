/***************************************************************************//**
* \file SERIAL_UART.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
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


#if (SERIAL_UART_WAKE_ENABLE_CONST && SERIAL_UART_RX_WAKEUP_IRQ)
    /**
    * \addtogroup group_globals
    * \{
    */
    /** This global variable determines whether to enable Skip Start
    * functionality when SERIAL_Sleep() function is called:
    * 0 – disable, other values – enable. Default value is 1.
    * It is only available when Enable wakeup from Deep Sleep Mode is enabled.
    */
    uint8 SERIAL_skipStart = 1u;
    /** \} globals */
#endif /* (SERIAL_UART_WAKE_ENABLE_CONST && SERIAL_UART_RX_WAKEUP_IRQ) */

#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const SERIAL_UART_INIT_STRUCT SERIAL_configUart =
    {
        SERIAL_UART_SUB_MODE,
        SERIAL_UART_DIRECTION,
        SERIAL_UART_DATA_BITS_NUM,
        SERIAL_UART_PARITY_TYPE,
        SERIAL_UART_STOP_BITS_NUM,
        SERIAL_UART_OVS_FACTOR,
        SERIAL_UART_IRDA_LOW_POWER,
        SERIAL_UART_MEDIAN_FILTER_ENABLE,
        SERIAL_UART_RETRY_ON_NACK,
        SERIAL_UART_IRDA_POLARITY,
        SERIAL_UART_DROP_ON_PARITY_ERR,
        SERIAL_UART_DROP_ON_FRAME_ERR,
        SERIAL_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        SERIAL_UART_MP_MODE_ENABLE,
        SERIAL_UART_MP_ACCEPT_ADDRESS,
        SERIAL_UART_MP_RX_ADDRESS,
        SERIAL_UART_MP_RX_ADDRESS_MASK,
        (uint32) SERIAL_SCB_IRQ_INTERNAL,
        SERIAL_UART_INTR_RX_MASK,
        SERIAL_UART_RX_TRIGGER_LEVEL,
        SERIAL_UART_INTR_TX_MASK,
        SERIAL_UART_TX_TRIGGER_LEVEL,
        (uint8) SERIAL_UART_BYTE_MODE_ENABLE,
        (uint8) SERIAL_UART_CTS_ENABLE,
        (uint8) SERIAL_UART_CTS_POLARITY,
        (uint8) SERIAL_UART_RTS_POLARITY,
        (uint8) SERIAL_UART_RTS_FIFO_LEVEL,
        (uint8) SERIAL_UART_RX_BREAK_WIDTH
    };


    /*******************************************************************************
    * Function Name: SERIAL_UartInit
    ****************************************************************************//**
    *
    *  Configures the SERIAL for UART operation.
    *
    *  This function is intended specifically to be used when the SERIAL
    *  configuration is set to “Unconfigured SERIAL” in the customizer.
    *  After initializing the SERIAL in UART mode using this function,
    *  the component can be enabled using the SERIAL_Start() or
    * SERIAL_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration
    *  settings. This structure contains the same information that would otherwise
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of
    *   fields. These fields match the selections available in the customizer.
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void SERIAL_UartInit(const SERIAL_UART_INIT_STRUCT *config)
    {
        uint32 pinsConfig;

        if (NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Get direction to configure UART pins: TX, RX or TX+RX */
            pinsConfig  = config->direction;

        #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
            /* Add RTS and CTS pins to configure */
            pinsConfig |= (0u != config->rtsRxFifoLevel) ? (SERIAL_UART_RTS_PIN_ENABLE) : (0u);
            pinsConfig |= (0u != config->enableCts)      ? (SERIAL_UART_CTS_PIN_ENABLE) : (0u);
        #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */

            /* Configure pins */
            SERIAL_SetPins(SERIAL_SCB_MODE_UART, config->mode, pinsConfig);

            /* Store internal configuration */
            SERIAL_scbMode       = (uint8) SERIAL_SCB_MODE_UART;
            SERIAL_scbEnableWake = (uint8) config->enableWake;
            SERIAL_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            SERIAL_rxBuffer      =         config->rxBuffer;
            SERIAL_rxDataBits    = (uint8) config->dataBits;
            SERIAL_rxBufferSize  =         config->rxBufferSize;

            /* Set TX direction internal variables */
            SERIAL_txBuffer      =         config->txBuffer;
            SERIAL_txDataBits    = (uint8) config->dataBits;
            SERIAL_txBufferSize  =         config->txBufferSize;

            /* Configure UART interface */
            if(SERIAL_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                SERIAL_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (SERIAL_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (SERIAL_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                SERIAL_CTRL_REG  = SERIAL_GET_CTRL_OVS(config->oversample);
            }

            SERIAL_CTRL_REG     |= SERIAL_GET_CTRL_BYTE_MODE  (config->enableByteMode)      |
                                             SERIAL_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             SERIAL_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            SERIAL_UART_CTRL_REG = SERIAL_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            SERIAL_UART_RX_CTRL_REG = SERIAL_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        SERIAL_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        SERIAL_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        SERIAL_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        SERIAL_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr)   |
                                        SERIAL_GET_UART_RX_CTRL_BREAK_WIDTH(config->breakWidth);

            if(SERIAL_UART_PARITY_NONE != config->parity)
            {
               SERIAL_UART_RX_CTRL_REG |= SERIAL_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    SERIAL_UART_RX_CTRL_PARITY_ENABLED;
            }

            SERIAL_RX_CTRL_REG      = SERIAL_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                SERIAL_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                SERIAL_GET_UART_RX_CTRL_ENABLED(config->direction);

            SERIAL_RX_FIFO_CTRL_REG = SERIAL_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            SERIAL_RX_MATCH_REG     = SERIAL_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                SERIAL_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            SERIAL_UART_TX_CTRL_REG = SERIAL_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                SERIAL_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(SERIAL_UART_PARITY_NONE != config->parity)
            {
               SERIAL_UART_TX_CTRL_REG |= SERIAL_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    SERIAL_UART_TX_CTRL_PARITY_ENABLED;
            }

            SERIAL_TX_CTRL_REG      = SERIAL_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                SERIAL_GET_UART_TX_CTRL_ENABLED(config->direction);

            SERIAL_TX_FIFO_CTRL_REG = SERIAL_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

        #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
            SERIAL_UART_FLOW_CTRL_REG = SERIAL_GET_UART_FLOW_CTRL_CTS_ENABLE(config->enableCts) | \
                                            SERIAL_GET_UART_FLOW_CTRL_CTS_POLARITY (config->ctsPolarity)  | \
                                            SERIAL_GET_UART_FLOW_CTRL_RTS_POLARITY (config->rtsPolarity)  | \
                                            SERIAL_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(config->rtsRxFifoLevel);
        #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */

            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (SERIAL_ISR_NUMBER);
            CyIntSetPriority(SERIAL_ISR_NUMBER, SERIAL_ISR_PRIORITY);
            (void) CyIntSetVector(SERIAL_ISR_NUMBER, &SERIAL_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(SERIAL_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (SERIAL_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(SERIAL_RX_WAKE_ISR_NUMBER, SERIAL_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(SERIAL_RX_WAKE_ISR_NUMBER, &SERIAL_UART_WAKEUP_ISR);
        #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            SERIAL_INTR_I2C_EC_MASK_REG = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_SPI_EC_MASK_REG = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_SLAVE_MASK_REG  = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_MASTER_MASK_REG = SERIAL_NO_INTR_SOURCES;
            SERIAL_INTR_RX_MASK_REG     = config->rxInterruptMask;
            SERIAL_INTR_TX_MASK_REG     = config->txInterruptMask;

            /* Configure TX interrupt sources to restore. */
            SERIAL_IntrTxMask = LO16(SERIAL_INTR_TX_MASK_REG);

            /* Clear RX buffer indexes */
            SERIAL_rxBufferHead     = 0u;
            SERIAL_rxBufferTail     = 0u;
            SERIAL_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            SERIAL_txBufferHead = 0u;
            SERIAL_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: SERIAL_UartInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the UART operation.
    *
    *******************************************************************************/
    void SERIAL_UartInit(void)
    {
        /* Configure UART interface */
        SERIAL_CTRL_REG = SERIAL_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        SERIAL_UART_CTRL_REG = SERIAL_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        SERIAL_UART_RX_CTRL_REG = SERIAL_UART_DEFAULT_UART_RX_CTRL;
        SERIAL_RX_CTRL_REG      = SERIAL_UART_DEFAULT_RX_CTRL;
        SERIAL_RX_FIFO_CTRL_REG = SERIAL_UART_DEFAULT_RX_FIFO_CTRL;
        SERIAL_RX_MATCH_REG     = SERIAL_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        SERIAL_UART_TX_CTRL_REG = SERIAL_UART_DEFAULT_UART_TX_CTRL;
        SERIAL_TX_CTRL_REG      = SERIAL_UART_DEFAULT_TX_CTRL;
        SERIAL_TX_FIFO_CTRL_REG = SERIAL_UART_DEFAULT_TX_FIFO_CTRL;

    #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
        SERIAL_UART_FLOW_CTRL_REG = SERIAL_UART_DEFAULT_FLOW_CTRL;
    #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */

        /* Configure interrupt with UART handler but do not enable it */
    #if(SERIAL_SCB_IRQ_INTERNAL)
        CyIntDisable    (SERIAL_ISR_NUMBER);
        CyIntSetPriority(SERIAL_ISR_NUMBER, SERIAL_ISR_PRIORITY);
        (void) CyIntSetVector(SERIAL_ISR_NUMBER, &SERIAL_SPI_UART_ISR);
    #endif /* (SERIAL_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(SERIAL_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (SERIAL_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(SERIAL_RX_WAKE_ISR_NUMBER, SERIAL_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(SERIAL_RX_WAKE_ISR_NUMBER, &SERIAL_UART_WAKEUP_ISR);
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        SERIAL_INTR_I2C_EC_MASK_REG = SERIAL_UART_DEFAULT_INTR_I2C_EC_MASK;
        SERIAL_INTR_SPI_EC_MASK_REG = SERIAL_UART_DEFAULT_INTR_SPI_EC_MASK;
        SERIAL_INTR_SLAVE_MASK_REG  = SERIAL_UART_DEFAULT_INTR_SLAVE_MASK;
        SERIAL_INTR_MASTER_MASK_REG = SERIAL_UART_DEFAULT_INTR_MASTER_MASK;
        SERIAL_INTR_RX_MASK_REG     = SERIAL_UART_DEFAULT_INTR_RX_MASK;
        SERIAL_INTR_TX_MASK_REG     = SERIAL_UART_DEFAULT_INTR_TX_MASK;

        /* Configure TX interrupt sources to restore. */
        SERIAL_IntrTxMask = LO16(SERIAL_INTR_TX_MASK_REG);

    #if(SERIAL_INTERNAL_RX_SW_BUFFER_CONST)
        SERIAL_rxBufferHead     = 0u;
        SERIAL_rxBufferTail     = 0u;
        SERIAL_rxBufferOverflow = 0u;
    #endif /* (SERIAL_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(SERIAL_INTERNAL_TX_SW_BUFFER_CONST)
        SERIAL_txBufferHead = 0u;
        SERIAL_txBufferTail = 0u;
    #endif /* (SERIAL_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: SERIAL_UartPostEnable
****************************************************************************//**
*
*  Restores HSIOM settings for the UART output pins (TX and/or RTS) to be
*  controlled by the SCB UART.
*
*******************************************************************************/
void SERIAL_UartPostEnable(void)
{
#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (SERIAL_TX_SDA_MISO_PIN)
        if (SERIAL_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set SCB UART to drive the output pin */
            SERIAL_SET_HSIOM_SEL(SERIAL_TX_SDA_MISO_HSIOM_REG, SERIAL_TX_SDA_MISO_HSIOM_MASK,
                                           SERIAL_TX_SDA_MISO_HSIOM_POS, SERIAL_TX_SDA_MISO_HSIOM_SEL_UART);
        }
    #endif /* (SERIAL_TX_SDA_MISO_PIN_PIN) */

    #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
        #if (SERIAL_SS0_PIN)
            if (SERIAL_CHECK_SS0_PIN_USED)
            {
                /* Set SCB UART to drive the output pin */
                SERIAL_SET_HSIOM_SEL(SERIAL_SS0_HSIOM_REG, SERIAL_SS0_HSIOM_MASK,
                                               SERIAL_SS0_HSIOM_POS, SERIAL_SS0_HSIOM_SEL_UART);
            }
        #endif /* (SERIAL_SS0_PIN) */
    #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */

#else
    #if (SERIAL_UART_TX_PIN)
         /* Set SCB UART to drive the output pin */
        SERIAL_SET_HSIOM_SEL(SERIAL_TX_HSIOM_REG, SERIAL_TX_HSIOM_MASK,
                                       SERIAL_TX_HSIOM_POS, SERIAL_TX_HSIOM_SEL_UART);
    #endif /* (SERIAL_UART_TX_PIN) */

    #if (SERIAL_UART_RTS_PIN)
        /* Set SCB UART to drive the output pin */
        SERIAL_SET_HSIOM_SEL(SERIAL_RTS_HSIOM_REG, SERIAL_RTS_HSIOM_MASK,
                                       SERIAL_RTS_HSIOM_POS, SERIAL_RTS_HSIOM_SEL_UART);
    #endif /* (SERIAL_UART_RTS_PIN) */
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Restore TX interrupt sources. */
    SERIAL_SetTxInterruptMode(SERIAL_IntrTxMask);
}


/*******************************************************************************
* Function Name: SERIAL_UartStop
****************************************************************************//**
*
*  Changes the HSIOM settings for the UART output pins (TX and/or RTS) to keep
*  them inactive after the block is disabled. The output pins are controlled by
*  the GPIO data register. Also, the function disables the skip start feature
*  to not cause it to trigger after the component is enabled.
*
*******************************************************************************/
void SERIAL_UartStop(void)
{
#if(SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (SERIAL_TX_SDA_MISO_PIN)
        if (SERIAL_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set GPIO to drive output pin */
            SERIAL_SET_HSIOM_SEL(SERIAL_TX_SDA_MISO_HSIOM_REG, SERIAL_TX_SDA_MISO_HSIOM_MASK,
                                           SERIAL_TX_SDA_MISO_HSIOM_POS, SERIAL_TX_SDA_MISO_HSIOM_SEL_GPIO);
        }
    #endif /* (SERIAL_TX_SDA_MISO_PIN_PIN) */

    #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
        #if (SERIAL_SS0_PIN)
            if (SERIAL_CHECK_SS0_PIN_USED)
            {
                /* Set output pin state after block is disabled */
                SERIAL_spi_ss0_Write(SERIAL_GET_UART_RTS_INACTIVE);

                /* Set GPIO to drive output pin */
                SERIAL_SET_HSIOM_SEL(SERIAL_SS0_HSIOM_REG, SERIAL_SS0_HSIOM_MASK,
                                               SERIAL_SS0_HSIOM_POS, SERIAL_SS0_HSIOM_SEL_GPIO);
            }
        #endif /* (SERIAL_SS0_PIN) */
    #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */

#else
    #if (SERIAL_UART_TX_PIN)
        /* Set GPIO to drive output pin */
        SERIAL_SET_HSIOM_SEL(SERIAL_TX_HSIOM_REG, SERIAL_TX_HSIOM_MASK,
                                       SERIAL_TX_HSIOM_POS, SERIAL_TX_HSIOM_SEL_GPIO);
    #endif /* (SERIAL_UART_TX_PIN) */

    #if (SERIAL_UART_RTS_PIN)
        /* Set output pin state after block is disabled */
        SERIAL_rts_Write(SERIAL_GET_UART_RTS_INACTIVE);

        /* Set GPIO to drive output pin */
        SERIAL_SET_HSIOM_SEL(SERIAL_RTS_HSIOM_REG, SERIAL_RTS_HSIOM_MASK,
                                       SERIAL_RTS_HSIOM_POS, SERIAL_RTS_HSIOM_SEL_GPIO);
    #endif /* (SERIAL_UART_RTS_PIN) */

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (SERIAL_UART_WAKE_ENABLE_CONST)
    /* Disable skip start feature used for wakeup */
    SERIAL_UART_RX_CTRL_REG &= (uint32) ~SERIAL_UART_RX_CTRL_SKIP_START;
#endif /* (SERIAL_UART_WAKE_ENABLE_CONST) */

    /* Store TX interrupt sources (exclude level triggered). */
    SERIAL_IntrTxMask = LO16(SERIAL_GetTxInterruptMode() & SERIAL_INTR_UART_TX_RESTORE);
}


/*******************************************************************************
* Function Name: SERIAL_UartSetRxAddress
****************************************************************************//**
*
*  Sets the hardware detectable receiver address for the UART in the
*  Multiprocessor mode.
*
*  \param address: Address for hardware address detection.
*
*******************************************************************************/
void SERIAL_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = SERIAL_RX_MATCH_REG;

    matchReg &= ((uint32) ~SERIAL_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & SERIAL_RX_MATCH_ADDR_MASK)); /* Set address  */

    SERIAL_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: SERIAL_UartSetRxAddressMask
****************************************************************************//**
*
*  Sets the hardware address mask for the UART in the Multiprocessor mode.
*
*  \param addressMask: Address mask.
*   - Bit value 0 – excludes bit from address comparison.
*   - Bit value 1 – the bit needs to match with the corresponding bit
*     of the address.
*
*******************************************************************************/
void SERIAL_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = SERIAL_RX_MATCH_REG;

    matchReg &= ((uint32) ~SERIAL_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << SERIAL_RX_MATCH_MASK_POS));

    SERIAL_RX_MATCH_REG = matchReg;
}


#if(SERIAL_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: SERIAL_UartGetChar
    ****************************************************************************//**
    *
    *  Retrieves next data element from receive buffer.
    *  This function is designed for ASCII characters and returns a char where
    *  1 to 255 are valid characters and 0 indicates an error occurred or no data
    *  is present.
    *  - RX software buffer is disabled: Returns data element retrieved from RX
    *    FIFO.
    *  - RX software buffer is enabled: Returns data element from the software
    *    receive buffer.
    *
    *  \return
    *   Next data element from the receive buffer. ASCII character values from
    *   1 to 255 are valid. A returned zero signifies an error condition or no
    *   data available.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check SERIAL_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 SERIAL_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Reads data only if there is data to read */
        if (0u != SERIAL_SpiUartGetRxBufferSize())
        {
            rxData = SERIAL_SpiUartReadRxData();
        }

        if (SERIAL_CHECK_INTR_RX(SERIAL_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: returns zero */
            SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_ERR);
        }

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: SERIAL_UartGetByte
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer, returns the
    *  received byte and error condition.
    *   - The RX software buffer is disabled: returns the data element retrieved
    *     from the RX FIFO. Undefined data will be returned if the RX FIFO is
    *     empty.
    *   - The RX software buffer is enabled: returns data element from the
    *     software receive buffer.
    *
    *  \return
    *   Bits 7-0 contain the next data element from the receive buffer and
    *   other bits contain the error condition.
    *   - SERIAL_UART_RX_OVERFLOW - Attempt to write to a full
    *     receiver FIFO.
    *   - SERIAL_UART_RX_UNDERFLOW    Attempt to read from an empty
    *     receiver FIFO.
    *   - SERIAL_UART_RX_FRAME_ERROR - UART framing error detected.
    *   - SERIAL_UART_RX_PARITY_ERROR - UART parity error detected.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check SERIAL_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 SERIAL_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;

        #if (SERIAL_CHECK_RX_SW_BUFFER)
        {
            SERIAL_DisableInt();
        }
        #endif

        if (0u != SERIAL_SpiUartGetRxBufferSize())
        {
            /* Enables interrupt to receive more bytes: at least one byte is in
            * buffer.
            */
            #if (SERIAL_CHECK_RX_SW_BUFFER)
            {
                SERIAL_EnableInt();
            }
            #endif

            /* Get received byte */
            rxData = SERIAL_SpiUartReadRxData();
        }
        else
        {
            /* Reads a byte directly from RX FIFO: underflow is raised in the
            * case of empty. Otherwise the first received byte will be read.
            */
            rxData = SERIAL_RX_FIFO_RD_REG;


            /* Enables interrupt to receive more bytes. */
            #if (SERIAL_CHECK_RX_SW_BUFFER)
            {

                /* The byte has been read from RX FIFO. Clear RX interrupt to
                * not involve interrupt handler when RX FIFO is empty.
                */
                SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_NOT_EMPTY);

                SERIAL_EnableInt();
            }
            #endif
        }

        /* Get and clear RX error mask */
        tmpStatus = (SERIAL_GetRxInterruptSource() & SERIAL_INTR_RX_ERR);
        SERIAL_ClearRxInterruptSource(SERIAL_INTR_RX_ERR);

        /* Puts together data and error status:
        * MP mode and accept address: 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return (rxData);
    }


    #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: SERIAL_UartSetRtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of RTS output signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param polarity: Active polarity of RTS output signal.
        *   - SERIAL_UART_RTS_ACTIVE_LOW  - RTS signal is active low.
        *   - SERIAL_UART_RTS_ACTIVE_HIGH - RTS signal is active high.
        *
        *******************************************************************************/
        void SERIAL_UartSetRtsPolarity(uint32 polarity)
        {
            if(0u != polarity)
            {
                SERIAL_UART_FLOW_CTRL_REG |= (uint32)  SERIAL_UART_FLOW_CTRL_RTS_POLARITY;
            }
            else
            {
                SERIAL_UART_FLOW_CTRL_REG &= (uint32) ~SERIAL_UART_FLOW_CTRL_RTS_POLARITY;
            }
        }


        /*******************************************************************************
        * Function Name: SERIAL_UartSetRtsFifoLevel
        ****************************************************************************//**
        *
        *  Sets level in the RX FIFO for RTS signal activation.
        *  While the RX FIFO has fewer entries than the RX FIFO level the RTS signal
        *  remains active, otherwise the RTS signal becomes inactive.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param level: Level in the RX FIFO for RTS signal activation.
        *   The range of valid level values is between 0 and RX FIFO depth - 1.
        *   Setting level value to 0 disables RTS signal activation.
        *
        *******************************************************************************/
        void SERIAL_UartSetRtsFifoLevel(uint32 level)
        {
            uint32 uartFlowCtrl;

            uartFlowCtrl = SERIAL_UART_FLOW_CTRL_REG;

            uartFlowCtrl &= ((uint32) ~SERIAL_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
            uartFlowCtrl |= ((uint32) (SERIAL_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK & level));

            SERIAL_UART_FLOW_CTRL_REG = uartFlowCtrl;
        }
    #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */

#endif /* (SERIAL_UART_RX_DIRECTION) */


#if(SERIAL_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: SERIAL_UartPutString
    ****************************************************************************//**
    *
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is a space available to put
    *  requested data in transmit buffer.
    *
    *  \param string: pointer to the null terminated string array to be placed in the
    *   transmit buffer.
    *
    *******************************************************************************/
    void SERIAL_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            SERIAL_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: SERIAL_UartPutCRLF
    ****************************************************************************//**
    *
    *  Places byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) in the transmit buffer.
    *  This function is blocking and waits until there is a space available to put
    *  all requested data in transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *
    *******************************************************************************/
    void SERIAL_UartPutCRLF(uint32 txDataByte)
    {
        SERIAL_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        SERIAL_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        SERIAL_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }


    #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: SERIALSCB_UartEnableCts
        ****************************************************************************//**
        *
        *  Enables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void SERIAL_UartEnableCts(void)
        {
            SERIAL_UART_FLOW_CTRL_REG |= (uint32)  SERIAL_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: SERIAL_UartDisableCts
        ****************************************************************************//**
        *
        *  Disables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void SERIAL_UartDisableCts(void)
        {
            SERIAL_UART_FLOW_CTRL_REG &= (uint32) ~SERIAL_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: SERIAL_UartSetCtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of CTS input signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        * \param
        * polarity: Active polarity of CTS output signal.
        *   - SERIAL_UART_CTS_ACTIVE_LOW  - CTS signal is active low.
        *   - SERIAL_UART_CTS_ACTIVE_HIGH - CTS signal is active high.
        *
        *******************************************************************************/
        void SERIAL_UartSetCtsPolarity(uint32 polarity)
        {
            if (0u != polarity)
            {
                SERIAL_UART_FLOW_CTRL_REG |= (uint32)  SERIAL_UART_FLOW_CTRL_CTS_POLARITY;
            }
            else
            {
                SERIAL_UART_FLOW_CTRL_REG &= (uint32) ~SERIAL_UART_FLOW_CTRL_CTS_POLARITY;
            }
        }
    #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */


    /*******************************************************************************
    * Function Name: SERIAL_UartSendBreakBlocking
    ****************************************************************************//**
    *
    * Sends a break condition (logic low) of specified width on UART TX line.
    * Blocks until break is completed. Only call this function when UART TX FIFO
    * and shifter are empty.
    *
    * \param breakWidth
    * Width of break condition. Valid range is 4 to 16 bits.
    *
    * \note
    * Before sending break all UART TX interrupt sources are disabled. The state
    * of UART TX interrupt sources is restored before function returns.
    *
    * \sideeffect
    * If this function is called while there is data in the TX FIFO or shifter that
    * data will be shifted out in packets the size of breakWidth.
    *
    *******************************************************************************/
    void SERIAL_UartSendBreakBlocking(uint32 breakWidth)
    {
        uint32 txCtrlReg;
        uint32 txIntrReg;

        /* Disable all UART TX interrupt source and clear UART TX Done history */
        txIntrReg = SERIAL_GetTxInterruptMode();
        SERIAL_SetTxInterruptMode(0u);
        SERIAL_ClearTxInterruptSource(SERIAL_INTR_TX_UART_DONE);

        /* Store TX CTRL configuration */
        txCtrlReg = SERIAL_TX_CTRL_REG;

        /* Set break width */
        SERIAL_TX_CTRL_REG = (SERIAL_TX_CTRL_REG & (uint32) ~SERIAL_TX_CTRL_DATA_WIDTH_MASK) |
                                        SERIAL_GET_TX_CTRL_DATA_WIDTH(breakWidth);

        /* Generate break */
        SERIAL_TX_FIFO_WR_REG = 0u;

        /* Wait for break completion */
        while (0u == (SERIAL_GetTxInterruptSource() & SERIAL_INTR_TX_UART_DONE))
        {
        }

        /* Clear all UART TX interrupt sources to  */
        SERIAL_ClearTxInterruptSource(SERIAL_INTR_TX_ALL);

        /* Restore TX interrupt sources and data width */
        SERIAL_TX_CTRL_REG = txCtrlReg;
        SERIAL_SetTxInterruptMode(txIntrReg);
    }
#endif /* (SERIAL_UART_TX_DIRECTION) */


#if (SERIAL_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: SERIAL_UartSaveConfig
    ****************************************************************************//**
    *
    *  Clears and enables an interrupt on a falling edge of the Rx input. The GPIO
    *  interrupt does not track in the active mode, therefore requires to be
    *  cleared by this API.
    *
    *******************************************************************************/
    void SERIAL_UartSaveConfig(void)
    {
    #if (SERIAL_UART_RX_WAKEUP_IRQ)
        /* Set SKIP_START if requested (set by default). */
        if (0u != SERIAL_skipStart)
        {
            SERIAL_UART_RX_CTRL_REG |= (uint32)  SERIAL_UART_RX_CTRL_SKIP_START;
        }
        else
        {
            SERIAL_UART_RX_CTRL_REG &= (uint32) ~SERIAL_UART_RX_CTRL_SKIP_START;
        }

        /* Clear RX GPIO interrupt status and pending interrupt in NVIC because
        * falling edge on RX line occurs while UART communication in active mode.
        * Enable interrupt: next interrupt trigger should wakeup device.
        */
        SERIAL_CLEAR_UART_RX_WAKE_INTR;
        SERIAL_RxWakeClearPendingInt();
        SERIAL_RxWakeEnableInt();
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: SERIAL_UartRestoreConfig
    ****************************************************************************//**
    *
    *  Disables the RX GPIO interrupt. Until this function is called the interrupt
    *  remains active and triggers on every falling edge of the UART RX line.
    *
    *******************************************************************************/
    void SERIAL_UartRestoreConfig(void)
    {
    #if (SERIAL_UART_RX_WAKEUP_IRQ)
        /* Disable interrupt: no more triggers in active mode */
        SERIAL_RxWakeDisableInt();
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */
    }


    #if (SERIAL_UART_RX_WAKEUP_IRQ)
        /*******************************************************************************
        * Function Name: SERIAL_UART_WAKEUP_ISR
        ****************************************************************************//**
        *
        *  Handles the Interrupt Service Routine for the SCB UART mode GPIO wakeup
        *  event. This event is configured to trigger on a falling edge of the RX line.
        *
        *******************************************************************************/
        CY_ISR(SERIAL_UART_WAKEUP_ISR)
        {
        #ifdef SERIAL_UART_WAKEUP_ISR_ENTRY_CALLBACK
            SERIAL_UART_WAKEUP_ISR_EntryCallback();
        #endif /* SERIAL_UART_WAKEUP_ISR_ENTRY_CALLBACK */

            SERIAL_CLEAR_UART_RX_WAKE_INTR;

        #ifdef SERIAL_UART_WAKEUP_ISR_EXIT_CALLBACK
            SERIAL_UART_WAKEUP_ISR_ExitCallback();
        #endif /* SERIAL_UART_WAKEUP_ISR_EXIT_CALLBACK */
        }
    #endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */
#endif /* (SERIAL_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
