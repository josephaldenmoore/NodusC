/*******************************************************************************
* File Name: Pin.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Pin_ALIASES_H) /* Pins Pin_ALIASES_H */
#define CY_PINS_Pin_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define Pin_0			(Pin__0__PC)
#define Pin_0_PS		(Pin__0__PS)
#define Pin_0_PC		(Pin__0__PC)
#define Pin_0_DR		(Pin__0__DR)
#define Pin_0_SHIFT	(Pin__0__SHIFT)
#define Pin_0_INTR	((uint16)((uint16)0x0003u << (Pin__0__SHIFT*2u)))

#define Pin_1			(Pin__1__PC)
#define Pin_1_PS		(Pin__1__PS)
#define Pin_1_PC		(Pin__1__PC)
#define Pin_1_DR		(Pin__1__DR)
#define Pin_1_SHIFT	(Pin__1__SHIFT)
#define Pin_1_INTR	((uint16)((uint16)0x0003u << (Pin__1__SHIFT*2u)))

#define Pin_2			(Pin__2__PC)
#define Pin_2_PS		(Pin__2__PS)
#define Pin_2_PC		(Pin__2__PC)
#define Pin_2_DR		(Pin__2__DR)
#define Pin_2_SHIFT	(Pin__2__SHIFT)
#define Pin_2_INTR	((uint16)((uint16)0x0003u << (Pin__2__SHIFT*2u)))

#define Pin_INTR_ALL	 ((uint16)(Pin_0_INTR| Pin_1_INTR| Pin_2_INTR))
#define Pin_RESISTOR			(Pin__RESISTOR__PC)
#define Pin_RESISTOR_PS		(Pin__RESISTOR__PS)
#define Pin_RESISTOR_PC		(Pin__RESISTOR__PC)
#define Pin_RESISTOR_DR		(Pin__RESISTOR__DR)
#define Pin_RESISTOR_SHIFT	(Pin__RESISTOR__SHIFT)
#define Pin_RESISTOR_INTR	((uint16)((uint16)0x0003u << (Pin__0__SHIFT*2u)))

#define Pin_SENSOR_1			(Pin__SENSOR_1__PC)
#define Pin_SENSOR_1_PS		(Pin__SENSOR_1__PS)
#define Pin_SENSOR_1_PC		(Pin__SENSOR_1__PC)
#define Pin_SENSOR_1_DR		(Pin__SENSOR_1__DR)
#define Pin_SENSOR_1_SHIFT	(Pin__SENSOR_1__SHIFT)
#define Pin_SENSOR_1_INTR	((uint16)((uint16)0x0003u << (Pin__1__SHIFT*2u)))

#define Pin_SENSOR_2			(Pin__SENSOR_2__PC)
#define Pin_SENSOR_2_PS		(Pin__SENSOR_2__PS)
#define Pin_SENSOR_2_PC		(Pin__SENSOR_2__PC)
#define Pin_SENSOR_2_DR		(Pin__SENSOR_2__DR)
#define Pin_SENSOR_2_SHIFT	(Pin__SENSOR_2__SHIFT)
#define Pin_SENSOR_2_INTR	((uint16)((uint16)0x0003u << (Pin__2__SHIFT*2u)))


#endif /* End Pins Pin_ALIASES_H */


/* [] END OF FILE */
