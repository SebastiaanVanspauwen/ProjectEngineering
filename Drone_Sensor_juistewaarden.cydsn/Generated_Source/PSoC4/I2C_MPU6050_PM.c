/***************************************************************************//**
* \file I2C_MPU6050_PM.c
* \version 4.0
*
* \brief
*  This file provides the source code to the Power Management support for
*  the SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "I2C_MPU6050.h"
#include "I2C_MPU6050_PVT.h"

#if(I2C_MPU6050_SCB_MODE_I2C_INC)
    #include "I2C_MPU6050_I2C_PVT.h"
#endif /* (I2C_MPU6050_SCB_MODE_I2C_INC) */

#if(I2C_MPU6050_SCB_MODE_EZI2C_INC)
    #include "I2C_MPU6050_EZI2C_PVT.h"
#endif /* (I2C_MPU6050_SCB_MODE_EZI2C_INC) */

#if(I2C_MPU6050_SCB_MODE_SPI_INC || I2C_MPU6050_SCB_MODE_UART_INC)
    #include "I2C_MPU6050_SPI_UART_PVT.h"
#endif /* (I2C_MPU6050_SCB_MODE_SPI_INC || I2C_MPU6050_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG || \
   (I2C_MPU6050_SCB_MODE_I2C_CONST_CFG   && (!I2C_MPU6050_I2C_WAKE_ENABLE_CONST))   || \
   (I2C_MPU6050_SCB_MODE_EZI2C_CONST_CFG && (!I2C_MPU6050_EZI2C_WAKE_ENABLE_CONST)) || \
   (I2C_MPU6050_SCB_MODE_SPI_CONST_CFG   && (!I2C_MPU6050_SPI_WAKE_ENABLE_CONST))   || \
   (I2C_MPU6050_SCB_MODE_UART_CONST_CFG  && (!I2C_MPU6050_UART_WAKE_ENABLE_CONST)))

    I2C_MPU6050_BACKUP_STRUCT I2C_MPU6050_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: I2C_MPU6050_Sleep
****************************************************************************//**
*
*  Prepares the I2C_MPU6050 component to enter Deep Sleep.
*  The �Enable wakeup from Deep Sleep Mode� selection has an influence on this 
*  function implementation:
*  - Checked: configures the component to be wakeup source from Deep Sleep.
*  - Unchecked: stores the current component state (enabled or disabled) and 
*    disables the component. See SCB_Stop() function for details about component 
*    disabling.
*
*  Call the I2C_MPU6050_Sleep() function before calling the 
*  CyPmSysDeepSleep() function. 
*  Refer to the PSoC Creator System Reference Guide for more information about 
*  power management functions and Low power section of this document for the 
*  selected mode.
*
*  This function should not be called before entering Sleep.
*
*******************************************************************************/
void I2C_MPU6050_Sleep(void)
{
#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)

    if(I2C_MPU6050_SCB_WAKE_ENABLE_CHECK)
    {
        if(I2C_MPU6050_SCB_MODE_I2C_RUNTM_CFG)
        {
            I2C_MPU6050_I2CSaveConfig();
        }
        else if(I2C_MPU6050_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            I2C_MPU6050_EzI2CSaveConfig();
        }
    #if(!I2C_MPU6050_CY_SCBIP_V1)
        else if(I2C_MPU6050_SCB_MODE_SPI_RUNTM_CFG)
        {
            I2C_MPU6050_SpiSaveConfig();
        }
        else if(I2C_MPU6050_SCB_MODE_UART_RUNTM_CFG)
        {
            I2C_MPU6050_UartSaveConfig();
        }
    #endif /* (!I2C_MPU6050_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        I2C_MPU6050_backup.enableState = (uint8) I2C_MPU6050_GET_CTRL_ENABLED;

        if(0u != I2C_MPU6050_backup.enableState)
        {
            I2C_MPU6050_Stop();
        }
    }

#else

    #if (I2C_MPU6050_SCB_MODE_I2C_CONST_CFG && I2C_MPU6050_I2C_WAKE_ENABLE_CONST)
        I2C_MPU6050_I2CSaveConfig();

    #elif (I2C_MPU6050_SCB_MODE_EZI2C_CONST_CFG && I2C_MPU6050_EZI2C_WAKE_ENABLE_CONST)
        I2C_MPU6050_EzI2CSaveConfig();

    #elif (I2C_MPU6050_SCB_MODE_SPI_CONST_CFG && I2C_MPU6050_SPI_WAKE_ENABLE_CONST)
        I2C_MPU6050_SpiSaveConfig();

    #elif (I2C_MPU6050_SCB_MODE_UART_CONST_CFG && I2C_MPU6050_UART_WAKE_ENABLE_CONST)
        I2C_MPU6050_UartSaveConfig();

    #else

        I2C_MPU6050_backup.enableState = (uint8) I2C_MPU6050_GET_CTRL_ENABLED;

        if(0u != I2C_MPU6050_backup.enableState)
        {
            I2C_MPU6050_Stop();
        }

    #endif /* defined (I2C_MPU6050_SCB_MODE_I2C_CONST_CFG) && (I2C_MPU6050_I2C_WAKE_ENABLE_CONST) */

#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: I2C_MPU6050_Wakeup
****************************************************************************//**
*
*  Prepares the I2C_MPU6050 component for Active mode operation after 
*  Deep Sleep.
*  The �Enable wakeup from Deep Sleep Mode� selection has influence on this 
*  function implementation:
*  - Checked: restores the component Active mode configuration.
*  - Unchecked: enables the component if it was enabled before enter Deep Sleep.
*
*  This function should not be called after exiting Sleep.
*
*  \sideeffect
*   Calling the I2C_MPU6050_Wakeup() function without first calling the 
*   I2C_MPU6050_Sleep() function may produce unexpected behavior.
*
*******************************************************************************/
void I2C_MPU6050_Wakeup(void)
{
#if(I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG)

    if(I2C_MPU6050_SCB_WAKE_ENABLE_CHECK)
    {
        if(I2C_MPU6050_SCB_MODE_I2C_RUNTM_CFG)
        {
            I2C_MPU6050_I2CRestoreConfig();
        }
        else if(I2C_MPU6050_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            I2C_MPU6050_EzI2CRestoreConfig();
        }
    #if(!I2C_MPU6050_CY_SCBIP_V1)
        else if(I2C_MPU6050_SCB_MODE_SPI_RUNTM_CFG)
        {
            I2C_MPU6050_SpiRestoreConfig();
        }
        else if(I2C_MPU6050_SCB_MODE_UART_RUNTM_CFG)
        {
            I2C_MPU6050_UartRestoreConfig();
        }
    #endif /* (!I2C_MPU6050_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != I2C_MPU6050_backup.enableState)
        {
            I2C_MPU6050_Enable();
        }
    }

#else

    #if (I2C_MPU6050_SCB_MODE_I2C_CONST_CFG  && I2C_MPU6050_I2C_WAKE_ENABLE_CONST)
        I2C_MPU6050_I2CRestoreConfig();

    #elif (I2C_MPU6050_SCB_MODE_EZI2C_CONST_CFG && I2C_MPU6050_EZI2C_WAKE_ENABLE_CONST)
        I2C_MPU6050_EzI2CRestoreConfig();

    #elif (I2C_MPU6050_SCB_MODE_SPI_CONST_CFG && I2C_MPU6050_SPI_WAKE_ENABLE_CONST)
        I2C_MPU6050_SpiRestoreConfig();

    #elif (I2C_MPU6050_SCB_MODE_UART_CONST_CFG && I2C_MPU6050_UART_WAKE_ENABLE_CONST)
        I2C_MPU6050_UartRestoreConfig();

    #else

        if(0u != I2C_MPU6050_backup.enableState)
        {
            I2C_MPU6050_Enable();
        }

    #endif /* (I2C_MPU6050_I2C_WAKE_ENABLE_CONST) */

#endif /* (I2C_MPU6050_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
