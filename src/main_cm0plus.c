/***************************************************************************//**
* \file main_cm0plus.c
*
* \version 1.0
*
* \brief Main example file for CM0+
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"
#include "tda803d_amp.h"
#include "tcan1145d_can.h"
#include "main_config.h"

//////////////////////////////////////////////////////////////////////////////
int main(void)
{
    SystemInit();
    
    __enable_irq(); /* Enable global interrupts. */
    
    // Example had been originally tested with "cache off", so ensure that caches are turned off (may have been enabled by new startup.c module)
    //SCB_DisableICache(); // Disables and invalidates instruction cache
    //SCB_DisableDCache(); // Disables, cleans and invalidates data cache

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
#ifdef ESTEC_SPI_ENABLE_FOR_CAN	
    Spi_Init(); //SPI
#endif
#ifdef ESTEC_CAN_ENABLE
    CAN_Init(); //CAN Init

    //current_can_mode = CAN_INITIAL_MODE;
#endif
#ifdef FDA806D_AMP_ENABLE
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	Fda803d_AmpInit();
#endif

    for(;;)
    {
    	Cy_SysLib_Delay(1000); //Call CAN transmission in every 1sec
#ifdef ESTEC_CAN_ENABLE
		CAN_Transceiver_Status(current_can_mode);
#endif
    }
}


/* [] END OF FILE */
