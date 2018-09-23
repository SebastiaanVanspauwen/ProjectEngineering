/***************************************************************************//**
* \file SERIAL.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component.
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

#if (SERIAL_SCB_MODE_I2C_INC)
    #include "SERIAL_I2C_PVT.h"
#endif /* (SERIAL_SCB_MODE_I2C_INC) */

#if (SERIAL_SCB_MODE_EZI2C_INC)
    #include "SERIAL_EZI2C_PVT.h"
#endif /* (SERIAL_SCB_MODE_EZI2C_INC) */

#if (SERIAL_SCB_MODE_SPI_INC || SERIAL_SCB_MODE_UART_INC)
    #include "SERIAL_SPI_UART_PVT.h"
#endif /* (SERIAL_SCB_MODE_SPI_INC || SERIAL_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 SERIAL_scbMode = SERIAL_SCB_MODE_UNCONFIG;
    uint8 SERIAL_scbEnableWake;
    uint8 SERIAL_scbEnableIntr;

    /* I2C configuration variables */
    uint8 SERIAL_mode;
    uint8 SERIAL_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * SERIAL_rxBuffer;
    uint8  SERIAL_rxDataBits;
    uint32 SERIAL_rxBufferSize;

    volatile uint8 * SERIAL_txBuffer;
    uint8  SERIAL_txDataBits;
    uint32 SERIAL_txBufferSize;

    /* EZI2C configuration variables */
    uint8 SERIAL_numberOfAddr;
    uint8 SERIAL_subAddrSize;
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/
/**
* \addtogroup group_general
* \{
*/

/** SERIAL_initVar indicates whether the SERIAL 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the SERIAL_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  SERIAL_Init() function can be called before the 
*  SERIAL_Start() or SERIAL_Enable() function.
*/
uint8 SERIAL_initVar = 0u;


#if (! (SERIAL_SCB_MODE_I2C_CONST_CFG || \
        SERIAL_SCB_MODE_EZI2C_CONST_CFG))
    /** This global variable stores TX interrupt sources after 
    * SERIAL_Stop() is called. Only these TX interrupt sources 
    * will be restored on a subsequent SERIAL_Enable() call.
    */
    uint16 SERIAL_IntrTxMask = 0u;
#endif /* (! (SERIAL_SCB_MODE_I2C_CONST_CFG || \
              SERIAL_SCB_MODE_EZI2C_CONST_CFG)) */
/** \} globals */

#if (SERIAL_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER)
    void (*SERIAL_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER) */
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void SERIAL_ScbEnableIntr(void);
static void SERIAL_ScbModeStop(void);
static void SERIAL_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: SERIAL_Init
****************************************************************************//**
*
*  Initializes the SERIAL component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  SERIAL_I2CInit, SERIAL_SpiInit, 
*  SERIAL_UartInit or SERIAL_EzI2CInit.
*
*******************************************************************************/
void SERIAL_Init(void)
{
#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    if (SERIAL_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        SERIAL_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (SERIAL_SCB_MODE_I2C_CONST_CFG)
    SERIAL_I2CInit();

#elif (SERIAL_SCB_MODE_SPI_CONST_CFG)
    SERIAL_SpiInit();

#elif (SERIAL_SCB_MODE_UART_CONST_CFG)
    SERIAL_UartInit();

#elif (SERIAL_SCB_MODE_EZI2C_CONST_CFG)
    SERIAL_EzI2CInit();

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_Enable
****************************************************************************//**
*
*  Enables SERIAL component operation: activates the hardware and 
*  internal interrupt. It also restores TX interrupt sources disabled after the 
*  SERIAL_Stop() function was called (note that level-triggered TX 
*  interrupt sources remain disabled to not cause code lock-up).
*  For I2C and EZI2C modes the interrupt is internal and mandatory for 
*  operation. For SPI and UART modes the interrupt can be configured as none, 
*  internal or external.
*  The SERIAL configuration should be not changed when the component
*  is enabled. Any configuration changes should be made after disabling the 
*  component.
*  When configuration is set to “Unconfigured SERIAL”, the component 
*  must first be initialized to operate in one of the following configurations: 
*  I2C, SPI, UART or EZ I2C, using the mode-specific initialization API. 
*  Otherwise this function does not enable the component.
*
*******************************************************************************/
void SERIAL_Enable(void)
{
#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!SERIAL_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        SERIAL_CTRL_REG |= SERIAL_CTRL_ENABLED;

        SERIAL_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        SERIAL_ScbModePostEnable();
    }
#else
    SERIAL_CTRL_REG |= SERIAL_CTRL_ENABLED;

    SERIAL_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    SERIAL_ScbModePostEnable();
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_Start
****************************************************************************//**
*
*  Invokes SERIAL_Init() and SERIAL_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZI2C. Otherwise this function does not enable the component.
*
* \globalvars
*  SERIAL_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void SERIAL_Start(void)
{
    if (0u == SERIAL_initVar)
    {
        SERIAL_Init();
        SERIAL_initVar = 1u; /* Component was initialized */
    }

    SERIAL_Enable();
}


/*******************************************************************************
* Function Name: SERIAL_Stop
****************************************************************************//**
*
*  Disables the SERIAL component: disable the hardware and internal 
*  interrupt. It also disables all TX interrupt sources so as not to cause an 
*  unexpected interrupt trigger because after the component is enabled, the 
*  TX FIFO is empty.
*  Refer to the function SERIAL_Enable() for the interrupt 
*  configuration details.
*  This function disables the SCB component without checking to see if 
*  communication is in progress. Before calling this function it may be 
*  necessary to check the status of communication to make sure communication 
*  is complete. If this is not done then communication could be stopped mid 
*  byte and corrupted data could result.
*
*******************************************************************************/
void SERIAL_Stop(void)
{
#if (SERIAL_SCB_IRQ_INTERNAL)
    SERIAL_DisableInt();
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    SERIAL_ScbModeStop();

    /* Disable SCB IP */
    SERIAL_CTRL_REG &= (uint32) ~SERIAL_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger after the component will be enabled because the 
    * TX FIFO is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when it is disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    SERIAL_SetTxInterruptMode(SERIAL_NO_INTR_SOURCES);

#if (SERIAL_SCB_IRQ_INTERNAL)
    SERIAL_ClearPendingInt();
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: SERIAL_SetRxFifoLevel
****************************************************************************//**
*
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
*  \param level: Level in the RX FIFO to generate RX level interrupt.
*   The range of valid level values is between 0 and RX FIFO depth - 1.
*
*******************************************************************************/
void SERIAL_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = SERIAL_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~SERIAL_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (SERIAL_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    SERIAL_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: SERIAL_SetTxFifoLevel
****************************************************************************//**
*
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has less entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
*  \param level: Level in the TX FIFO to generate TX level interrupt.
*   The range of valid level values is between 0 and TX FIFO depth - 1.
*
*******************************************************************************/
void SERIAL_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = SERIAL_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~SERIAL_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (SERIAL_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    SERIAL_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (SERIAL_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: SERIAL_SetCustomInterruptHandler
    ****************************************************************************//**
    *
    *  Registers a function to be called by the internal interrupt handler.
    *  First the function that is registered is called, then the internal interrupt
    *  handler performs any operation such as software buffer management functions
    *  before the interrupt returns.  It is the user's responsibility not to break
    *  the software buffer operations. Only one custom handler is supported, which
    *  is the function provided by the most recent call.
    *  At the initialization time no custom handler is registered.
    *
    *  \param func: Pointer to the function to register.
    *        The value NULL indicates to remove the current custom interrupt
    *        handler.
    *
    *******************************************************************************/
    void SERIAL_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER)
        SERIAL_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_SERIAL_CUSTOM_INTR_HANDLER) */
    }
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: SERIAL_ScbModeEnableIntr
****************************************************************************//**
*
*  Enables an interrupt for a specific mode.
*
*******************************************************************************/
static void SERIAL_ScbEnableIntr(void)
{
#if (SERIAL_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != SERIAL_scbEnableIntr)
        {
            SERIAL_EnableInt();
        }

    #else
        SERIAL_EnableInt();

    #endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (SERIAL_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: SERIAL_ScbModePostEnable
****************************************************************************//**
*
*  Calls the PostEnable function for a specific operation mode.
*
*******************************************************************************/
static void SERIAL_ScbModePostEnable(void)
{
#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!SERIAL_CY_SCBIP_V1)
    if (SERIAL_SCB_MODE_SPI_RUNTM_CFG)
    {
        SERIAL_SpiPostEnable();
    }
    else if (SERIAL_SCB_MODE_UART_RUNTM_CFG)
    {
        SERIAL_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!SERIAL_CY_SCBIP_V1) */

#elif (SERIAL_SCB_MODE_SPI_CONST_CFG)
    SERIAL_SpiPostEnable();

#elif (SERIAL_SCB_MODE_UART_CONST_CFG)
    SERIAL_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SERIAL_ScbModeStop
****************************************************************************//**
*
*  Calls the Stop function for a specific operation mode.
*
*******************************************************************************/
static void SERIAL_ScbModeStop(void)
{
#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    if (SERIAL_SCB_MODE_I2C_RUNTM_CFG)
    {
        SERIAL_I2CStop();
    }
    else if (SERIAL_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        SERIAL_EzI2CStop();
    }
#if (!SERIAL_CY_SCBIP_V1)
    else if (SERIAL_SCB_MODE_SPI_RUNTM_CFG)
    {
        SERIAL_SpiStop();
    }
    else if (SERIAL_SCB_MODE_UART_RUNTM_CFG)
    {
        SERIAL_UartStop();
    }
#endif /* (!SERIAL_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (SERIAL_SCB_MODE_I2C_CONST_CFG)
    SERIAL_I2CStop();

#elif (SERIAL_SCB_MODE_EZI2C_CONST_CFG)
    SERIAL_EzI2CStop();

#elif (SERIAL_SCB_MODE_SPI_CONST_CFG)
    SERIAL_SpiStop();

#elif (SERIAL_SCB_MODE_UART_CONST_CFG)
    SERIAL_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: SERIAL_SetPins
    ****************************************************************************//**
    *
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    *  \param mode:      Mode of SCB operation.
    *  \param subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  \param uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    *******************************************************************************/
    void SERIAL_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 pinsDm[SERIAL_SCB_PINS_NUMBER];
        uint32 i;
        
    #if (!SERIAL_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!SERIAL_CY_SCBIP_V1) */
        
        uint32 hsiomSel[SERIAL_SCB_PINS_NUMBER] = 
        {
            SERIAL_RX_SCL_MOSI_HSIOM_SEL_GPIO,
            SERIAL_TX_SDA_MISO_HSIOM_SEL_GPIO,
            0u,
            0u,
            0u,
            0u,
            0u,
        };

    #if (SERIAL_CY_SCBIP_V1)
        /* Supress compiler warning. */
        if ((0u == subMode) || (0u == uartEnableMask))
        {
        }
    #endif /* (SERIAL_CY_SCBIP_V1) */

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < SERIAL_SCB_PINS_NUMBER; i++)
        {
            pinsDm[i] = SERIAL_PIN_DM_ALG_HIZ;
        }

        if ((SERIAL_SCB_MODE_I2C   == mode) ||
            (SERIAL_SCB_MODE_EZI2C == mode))
        {
        #if (SERIAL_RX_SCL_MOSI_PIN)
            hsiomSel[SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_RX_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
        #elif (SERIAL_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX] = SERIAL_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
        #else
        #endif /* (SERIAL_RX_SCL_MOSI_PIN) */
        
        #if (SERIAL_TX_SDA_MISO_PIN)
            hsiomSel[SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_TX_SDA_MISO_HSIOM_SEL_I2C;
            pinsDm  [SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
        #endif /* (SERIAL_TX_SDA_MISO_PIN) */
        }
    #if (!SERIAL_CY_SCBIP_V1)
        else if (SERIAL_SCB_MODE_SPI == mode)
        {
        #if (SERIAL_RX_SCL_MOSI_PIN)
            hsiomSel[SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_RX_SCL_MOSI_HSIOM_SEL_SPI;
        #elif (SERIAL_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX] = SERIAL_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI;
        #else
        #endif /* (SERIAL_RX_SCL_MOSI_PIN) */
        
        #if (SERIAL_TX_SDA_MISO_PIN)
            hsiomSel[SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_TX_SDA_MISO_HSIOM_SEL_SPI;
        #endif /* (SERIAL_TX_SDA_MISO_PIN) */
        
        #if (SERIAL_SCLK_PIN)
            hsiomSel[SERIAL_SCLK_PIN_INDEX] = SERIAL_SCLK_HSIOM_SEL_SPI;
        #endif /* (SERIAL_SCLK_PIN) */

            if (SERIAL_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                pinsDm[SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsDm[SERIAL_SCLK_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;

            #if (SERIAL_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[SERIAL_SS0_PIN_INDEX] = SERIAL_SS0_HSIOM_SEL_SPI;
                pinsDm  [SERIAL_SS0_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
            #endif /* (SERIAL_SS0_PIN) */

            #if (SERIAL_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= SERIAL_TX_SDA_MISO_PIN_MASK;
            #endif /* (SERIAL_TX_SDA_MISO_PIN) */
            }
            else 
            {
                /* (Master) */
                pinsDm[SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsDm[SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                pinsDm[SERIAL_SCLK_PIN_INDEX] = SERIAL_PIN_DM_STRONG;

            #if (SERIAL_SS0_PIN)
                hsiomSel [SERIAL_SS0_PIN_INDEX] = SERIAL_SS0_HSIOM_SEL_SPI;
                pinsDm   [SERIAL_SS0_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf |= SERIAL_SS0_PIN_MASK;
            #endif /* (SERIAL_SS0_PIN) */

            #if (SERIAL_SS1_PIN)
                hsiomSel [SERIAL_SS1_PIN_INDEX] = SERIAL_SS1_HSIOM_SEL_SPI;
                pinsDm   [SERIAL_SS1_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf |= SERIAL_SS1_PIN_MASK;
            #endif /* (SERIAL_SS1_PIN) */

            #if (SERIAL_SS2_PIN)
                hsiomSel [SERIAL_SS2_PIN_INDEX] = SERIAL_SS2_HSIOM_SEL_SPI;
                pinsDm   [SERIAL_SS2_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf |= SERIAL_SS2_PIN_MASK;
            #endif /* (SERIAL_SS2_PIN) */

            #if (SERIAL_SS3_PIN)
                hsiomSel [SERIAL_SS3_PIN_INDEX] = SERIAL_SS3_HSIOM_SEL_SPI;
                pinsDm   [SERIAL_SS3_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                pinsInBuf |= SERIAL_SS3_PIN_MASK;
            #endif /* (SERIAL_SS3_PIN) */

                /* Disable input buffers */
            #if (SERIAL_RX_SCL_MOSI_PIN)
                pinsInBuf |= SERIAL_RX_SCL_MOSI_PIN_MASK;
            #elif (SERIAL_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= SERIAL_RX_WAKE_SCL_MOSI_PIN_MASK;
            #else
            #endif /* (SERIAL_RX_SCL_MOSI_PIN) */

            #if (SERIAL_SCLK_PIN)
                pinsInBuf |= SERIAL_SCLK_PIN_MASK;
            #endif /* (SERIAL_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (SERIAL_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
            #if (SERIAL_TX_SDA_MISO_PIN)
                hsiomSel[SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_TX_SDA_MISO_HSIOM_SEL_UART;
                pinsDm  [SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_PIN_DM_OD_LO;
            #endif /* (SERIAL_TX_SDA_MISO_PIN) */
            }
            else /* Standard or IrDA */
            {
                if (0u != (SERIAL_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                #if (SERIAL_RX_SCL_MOSI_PIN)
                    hsiomSel[SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_RX_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [SERIAL_RX_SCL_MOSI_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                #elif (SERIAL_RX_WAKE_SCL_MOSI_PIN)
                    hsiomSel[SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX] = SERIAL_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                #else
                #endif /* (SERIAL_RX_SCL_MOSI_PIN) */
                }

                if (0u != (SERIAL_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                #if (SERIAL_TX_SDA_MISO_PIN)
                    hsiomSel[SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_TX_SDA_MISO_HSIOM_SEL_UART;
                    pinsDm  [SERIAL_TX_SDA_MISO_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                    
                    /* Disable input buffer */
                    pinsInBuf |= SERIAL_TX_SDA_MISO_PIN_MASK;
                #endif /* (SERIAL_TX_SDA_MISO_PIN) */
                }

            #if !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
                if (SERIAL_UART_MODE_STD == subMode)
                {
                    if (0u != (SERIAL_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                    #if (SERIAL_SCLK_PIN)
                        hsiomSel[SERIAL_SCLK_PIN_INDEX] = SERIAL_SCLK_HSIOM_SEL_UART;
                        pinsDm  [SERIAL_SCLK_PIN_INDEX] = SERIAL_PIN_DM_DIG_HIZ;
                    #endif /* (SERIAL_SCLK_PIN) */
                    }

                    if (0u != (SERIAL_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                    #if (SERIAL_SS0_PIN)
                        hsiomSel[SERIAL_SS0_PIN_INDEX] = SERIAL_SS0_HSIOM_SEL_UART;
                        pinsDm  [SERIAL_SS0_PIN_INDEX] = SERIAL_PIN_DM_STRONG;
                        
                        /* Disable input buffer */
                        pinsInBuf |= SERIAL_SS0_PIN_MASK;
                    #endif /* (SERIAL_SS0_PIN) */
                    }
                }
            #endif /* !(SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */
            }
        }
    #endif /* (!SERIAL_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (SERIAL_RX_SCL_MOSI_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_RX_SCL_MOSI_HSIOM_REG,
                                       SERIAL_RX_SCL_MOSI_HSIOM_MASK,
                                       SERIAL_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[SERIAL_RX_SCL_MOSI_PIN_INDEX]);

        SERIAL_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[SERIAL_RX_SCL_MOSI_PIN_INDEX]);

        #if (!SERIAL_CY_SCBIP_V1)
            SERIAL_SET_INP_DIS(SERIAL_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                         SERIAL_uart_rx_i2c_scl_spi_mosi_MASK,
                                         (0u != (pinsInBuf & SERIAL_RX_SCL_MOSI_PIN_MASK)));
        #endif /* (!SERIAL_CY_SCBIP_V1) */
    
    #elif (SERIAL_RX_WAKE_SCL_MOSI_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       SERIAL_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       SERIAL_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        SERIAL_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[SERIAL_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     SERIAL_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & SERIAL_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        SERIAL_SET_INCFG_TYPE(SERIAL_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        SERIAL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        SERIAL_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        SERIAL_INTCFG_TYPE_FALLING_EDGE);
    #else
    #endif /* (SERIAL_RX_WAKE_SCL_MOSI_PIN) */

    #if (SERIAL_TX_SDA_MISO_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_TX_SDA_MISO_HSIOM_REG,
                                       SERIAL_TX_SDA_MISO_HSIOM_MASK,
                                       SERIAL_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[SERIAL_TX_SDA_MISO_PIN_INDEX]);

        SERIAL_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[SERIAL_TX_SDA_MISO_PIN_INDEX]);

    #if (!SERIAL_CY_SCBIP_V1)
        SERIAL_SET_INP_DIS(SERIAL_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     SERIAL_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & SERIAL_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!SERIAL_CY_SCBIP_V1) */
    #endif /* (SERIAL_RX_SCL_MOSI_PIN) */

    #if (SERIAL_SCLK_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SCLK_HSIOM_REG,
                                       SERIAL_SCLK_HSIOM_MASK,
                                       SERIAL_SCLK_HSIOM_POS,
                                       hsiomSel[SERIAL_SCLK_PIN_INDEX]);

        SERIAL_spi_sclk_SetDriveMode((uint8) pinsDm[SERIAL_SCLK_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_sclk_INP_DIS,
                                     SERIAL_spi_sclk_MASK,
                                     (0u != (pinsInBuf & SERIAL_SCLK_PIN_MASK)));
    #endif /* (SERIAL_SCLK_PIN) */

    #if (SERIAL_SS0_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS0_HSIOM_REG,
                                       SERIAL_SS0_HSIOM_MASK,
                                       SERIAL_SS0_HSIOM_POS,
                                       hsiomSel[SERIAL_SS0_PIN_INDEX]);

        SERIAL_spi_ss0_SetDriveMode((uint8) pinsDm[SERIAL_SS0_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss0_INP_DIS,
                                     SERIAL_spi_ss0_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS0_PIN_MASK)));
    #endif /* (SERIAL_SS0_PIN) */

    #if (SERIAL_SS1_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS1_HSIOM_REG,
                                       SERIAL_SS1_HSIOM_MASK,
                                       SERIAL_SS1_HSIOM_POS,
                                       hsiomSel[SERIAL_SS1_PIN_INDEX]);

        SERIAL_spi_ss1_SetDriveMode((uint8) pinsDm[SERIAL_SS1_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss1_INP_DIS,
                                     SERIAL_spi_ss1_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS1_PIN_MASK)));
    #endif /* (SERIAL_SS1_PIN) */

    #if (SERIAL_SS2_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS2_HSIOM_REG,
                                       SERIAL_SS2_HSIOM_MASK,
                                       SERIAL_SS2_HSIOM_POS,
                                       hsiomSel[SERIAL_SS2_PIN_INDEX]);

        SERIAL_spi_ss2_SetDriveMode((uint8) pinsDm[SERIAL_SS2_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss2_INP_DIS,
                                     SERIAL_spi_ss2_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS2_PIN_MASK)));
    #endif /* (SERIAL_SS2_PIN) */

    #if (SERIAL_SS3_PIN)
        SERIAL_SET_HSIOM_SEL(SERIAL_SS3_HSIOM_REG,
                                       SERIAL_SS3_HSIOM_MASK,
                                       SERIAL_SS3_HSIOM_POS,
                                       hsiomSel[SERIAL_SS3_PIN_INDEX]);

        SERIAL_spi_ss3_SetDriveMode((uint8) pinsDm[SERIAL_SS3_PIN_INDEX]);

        SERIAL_SET_INP_DIS(SERIAL_spi_ss3_INP_DIS,
                                     SERIAL_spi_ss3_MASK,
                                     (0u != (pinsInBuf & SERIAL_SS3_PIN_MASK)));
    #endif /* (SERIAL_SS3_PIN) */
    }

#endif /* (SERIAL_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: SERIAL_I2CSlaveNackGeneration
    ****************************************************************************//**
    *
    *  Sets command to generate NACK to the address or data.
    *
    *******************************************************************************/
    void SERIAL_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (SERIAL_CTRL_REG & SERIAL_CTRL_EC_AM_MODE)) &&
            (0u == (SERIAL_I2C_CTRL_REG & SERIAL_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            SERIAL_CTRL_REG &= ~SERIAL_CTRL_EC_AM_MODE;
            SERIAL_CTRL_REG |=  SERIAL_CTRL_EC_AM_MODE;
        }

        SERIAL_I2C_SLAVE_CMD_REG = SERIAL_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (SERIAL_CY_SCBIP_V0 || SERIAL_CY_SCBIP_V1) */


/* [] END OF FILE */
