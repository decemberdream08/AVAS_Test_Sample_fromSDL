/***************************************************************************//**
* \file tda803d_amp.h
*
* \version 1.0
*
* \brief Main example file for CM0+ & CM7_0
*
********************************************************************************
* \copyright
* ESTec KMS. This code is based on Cypress reference SW(SDL).
*******************************************************************************/

#ifndef _TCAN1145D_CAN_H_
#define _TCAN1145D_CAN_H_

//MACRO & Define ***********************************************************************/
typedef enum {
	CAN_INITIAL_MODE = (0u),
	CAN_NORMAL_MODE,
	CAN_STANDBY_MODE
} CAN_MODE;

extern CAN_MODE current_can_mode;

//Function Declaration ***********************************************************************/
void Spi_Init(void);

void CAN_Transceiver_Status(CAN_MODE cur_mode);
void CAN_Init(void);


#endif //_TCAN1145D_CAN_H_

