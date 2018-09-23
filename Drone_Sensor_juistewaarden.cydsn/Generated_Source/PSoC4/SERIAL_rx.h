/*******************************************************************************
* File Name: SERIAL_rx.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_SERIAL_rx_H) /* Pins SERIAL_rx_H */
#define CY_PINS_SERIAL_rx_H

#include "cytypes.h"
#include "cyfitter.h"
#include "SERIAL_rx_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} SERIAL_rx_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   SERIAL_rx_Read(void);
void    SERIAL_rx_Write(uint8 value);
uint8   SERIAL_rx_ReadDataReg(void);
#if defined(SERIAL_rx__PC) || (CY_PSOC4_4200L) 
    void    SERIAL_rx_SetDriveMode(uint8 mode);
#endif
void    SERIAL_rx_SetInterruptMode(uint16 position, uint16 mode);
uint8   SERIAL_rx_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void SERIAL_rx_Sleep(void); 
void SERIAL_rx_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(SERIAL_rx__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define SERIAL_rx_DRIVE_MODE_BITS        (3)
    #define SERIAL_rx_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - SERIAL_rx_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the SERIAL_rx_SetDriveMode() function.
         *  @{
         */
        #define SERIAL_rx_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define SERIAL_rx_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define SERIAL_rx_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define SERIAL_rx_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define SERIAL_rx_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define SERIAL_rx_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define SERIAL_rx_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define SERIAL_rx_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define SERIAL_rx_MASK               SERIAL_rx__MASK
#define SERIAL_rx_SHIFT              SERIAL_rx__SHIFT
#define SERIAL_rx_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in SERIAL_rx_SetInterruptMode() function.
     *  @{
     */
        #define SERIAL_rx_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define SERIAL_rx_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define SERIAL_rx_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define SERIAL_rx_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(SERIAL_rx__SIO)
    #define SERIAL_rx_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(SERIAL_rx__PC) && (CY_PSOC4_4200L)
    #define SERIAL_rx_USBIO_ENABLE               ((uint32)0x80000000u)
    #define SERIAL_rx_USBIO_DISABLE              ((uint32)(~SERIAL_rx_USBIO_ENABLE))
    #define SERIAL_rx_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define SERIAL_rx_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define SERIAL_rx_USBIO_ENTER_SLEEP          ((uint32)((1u << SERIAL_rx_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << SERIAL_rx_USBIO_SUSPEND_DEL_SHIFT)))
    #define SERIAL_rx_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << SERIAL_rx_USBIO_SUSPEND_SHIFT)))
    #define SERIAL_rx_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << SERIAL_rx_USBIO_SUSPEND_DEL_SHIFT)))
    #define SERIAL_rx_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(SERIAL_rx__PC)
    /* Port Configuration */
    #define SERIAL_rx_PC                 (* (reg32 *) SERIAL_rx__PC)
#endif
/* Pin State */
#define SERIAL_rx_PS                     (* (reg32 *) SERIAL_rx__PS)
/* Data Register */
#define SERIAL_rx_DR                     (* (reg32 *) SERIAL_rx__DR)
/* Input Buffer Disable Override */
#define SERIAL_rx_INP_DIS                (* (reg32 *) SERIAL_rx__PC2)

/* Interrupt configuration Registers */
#define SERIAL_rx_INTCFG                 (* (reg32 *) SERIAL_rx__INTCFG)
#define SERIAL_rx_INTSTAT                (* (reg32 *) SERIAL_rx__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define SERIAL_rx_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(SERIAL_rx__SIO)
    #define SERIAL_rx_SIO_REG            (* (reg32 *) SERIAL_rx__SIO)
#endif /* (SERIAL_rx__SIO_CFG) */

/* USBIO registers */
#if !defined(SERIAL_rx__PC) && (CY_PSOC4_4200L)
    #define SERIAL_rx_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define SERIAL_rx_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define SERIAL_rx_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define SERIAL_rx_DRIVE_MODE_SHIFT       (0x00u)
#define SERIAL_rx_DRIVE_MODE_MASK        (0x07u << SERIAL_rx_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins SERIAL_rx_H */


/* [] END OF FILE */
