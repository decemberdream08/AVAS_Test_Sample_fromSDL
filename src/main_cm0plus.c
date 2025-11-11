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
#include "main_config.h"
#ifdef FDA806D_AMP_ENABLE
#include "fda803d_amp.h"
#endif
#ifdef ESTEC_CAN_ENABLE
#include "tcan1145d_can.h"
#endif
#ifdef ESTEC_MCU_I2S_DW_ENABLE
#include "mcu_i2s_dw.h"
#endif


//////////////////////////////////////////////////////////////////////////////
int main(void)
{
    SystemInit();
    
    __enable_irq(); /* Enable global interrupts. */
    
    /* Enable CM7_0/1. CY_CORTEX_M7_APPL_ADDR is calculated in linker script, check it in case of problems. */
    //Cy_SysEnableApplCore(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);//DDD...TEST
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
#ifdef ESTEC_SPI_ENABLE_FOR_CAN	
    Spi_Init(); //SPI
#endif
#ifdef ESTEC_CAN_ENABLE
    CAN_Init(); //CAN Init

    //current_can_mode = CAN_INITIAL_MODE;
#endif
#ifdef FDA806D_AMP_ENABLE
	FDA803D_Amp_Init();
#endif
#ifdef ESTEC_MCU_I2S_DW_ENABLE
	I2S_Init();
#endif
#ifdef FDA806D_AMP_ENABLE
    FDA803D_Amp_Play();
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
