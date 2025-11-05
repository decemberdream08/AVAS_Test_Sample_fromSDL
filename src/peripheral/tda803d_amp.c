/***************************************************************************//**
* \file tda803d_amp.c
*
* \version 1.0
*
* \brief Main example file for CM0+ and CM7_0
*
********************************************************************************
* \copyright
* ESTec KMS. This code is based on Cypress reference SW(SDL).
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"
#include "tda803d_amp.h"
#include "main_config.h"

//MACRO & Define ***********************************************************************/
#ifdef FDA806D_AMP_ENABLE
#define DIVIDER_NO_1 (1u)

/* Select Frequency */
#if (CY_USE_PSVP == 1)
  #define E_SOURCE_CLK_FREQ     (24000000ul) // fixed
#else
  #define E_SOURCE_CLK_FREQ     (80000000u)  // fixed
#endif

#ifdef ESTEC_I2C_ENABLE
#define E_I2C_INCLK_TARGET_FREQ (2000000ul)  // modifiable
#define E_I2C_DATARATE          (100000ul)   // modifiable

#define USER_I2C_SCB_TYPE       SCB2
#define USER_I2C_SCB_PCLK       PCLK_SCB2_CLOCK
#define USER_I2C_SCB_IRQN       scb_2_interrupt_IRQn

#define I2C_SDA_PORT     		GPIO_PRT19
#define I2C_SDA_PORT_PIN 		(1)
#define I2C_SDA_PORT_MUX 		P19_1_SCB2_I2C_SDA

#define I2C_SCL_PORT     		GPIO_PRT19
#define I2C_SCL_PORT_PIN 		(2)
#define I2C_SCL_PORT_MUX 		P19_2_SCB2_I2C_SCL
#endif

#define AMP_I2C_SLAVE_ADDR               	0x70

#define FDA803D_IB1_REG                  	0x81
#define IB1_WS_FREQUENCY                 	0x40 //WS : 48kHz, PWM amplifier clock not dithered, PWM in phase

#define FDA803D_IB2_REG                  	0x82
#define IB2_FULL_SCALE_VOLTAGE_LIMIT_80  	0x0b

#define FDA803D_IB8_REG               		0x88
#define IB8_PWM_OFF                   		0x00
#define IB8_PWM_ON                    		0x20
#define IB8_CHANNEL_IN_MUTE           		0x00
#define IB8_CHANNEL_IN_PLAY           		0x01

#define FDA803D_DB2_REG               		0xA2
#define DB2_CHANNEL_IN_MUTE           		0x00
#define DB2_CHANNEL_IN_PLAY           		0x01

#ifdef ESTEC_GPIO_ENABLE
//Master Amp
//P18.6 - Output
#define AMP_MASTER_ON_PORT			GPIO_PRT18
#define AMP_MASTER_ON_PIN			6
#define AMP_MASTER_ON_PIN_MUX		P18_6_GPIO
//P18.5 - Output
#define AMP_MASTER_MUTE_PORT		GPIO_PRT18
#define AMP_MASTER_MUTE_PIN			5
#define AMP_MASTER_MUTE_PIN_MUX		P18_6_GPIO
//P18.7 - Input
#define AMP_MASTER_CDDIAG_PORT		GPIO_PRT18
#define AMP_MASTER_CDDIAG_PIN		7	
#define AMP_MASTER_CDDIAG_PIN_MUX		P18_7_GPIO

//Slave Amp
//P5.3 - Output
#define AMP_SLAVE_ON_PORT			GPIO_PRT5
#define AMP_SLAVE_ON_PIN			3
#define AMP_SLAVE_ON_PIN_MUX		P5_3_GPIO
//P5.2 - Output
#define AMP_SLAVE_MUTE_PORT			GPIO_PRT5
#define AMP_SLAVE_MUTE_PIN			2
#define AMP_SLAVE_MUTE_PIN_MUX		P5_2_GPIO
//P5.1 - Input
#define AMP_SLAVE_CDDIAG_PORT		GPIO_PRT5
#define AMP_SLAVE_CDDIAG_PIN		1	
#define AMP_SLAVE_CDDIAG_PIN_MUX		P5_1_GPIO

//CD_DIAG : MCU Input
//Need to make configuration later.

static cy_stc_gpio_pin_config_t user_amp_on_port_pin_cfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = AMP_MASTER_ON_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip = 0,                                    
    .slewRate = 0,                                 
    .driveSel = 0,                                 
    .vregEn = 0,                                   
    .ibufMode = 0,                                 
    .vtripSel = 0,                                 
    .vrefSel = 0,                                  
    .vohSel = 0,    
};
#endif //ESTEC_GPIO_ENABLE

#ifdef ESTEC_I2C_ENABLE
static cy_stc_gpio_pin_config_t i2c_port_pin_cfg =
{
    .outVal    = 0ul,
    .driveMode = 0ul,            /* Will be updated in runtime */
    .hsiom     = HSIOM_SEL_GPIO, /* Will be updated in runtime */
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

static cy_stc_sysint_irq_t i2c_irq_cfg =
{
    .sysIntSrc  = USER_I2C_SCB_IRQN,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

/* SCB - I2C Configuration */
static cy_stc_scb_i2c_context_t g_stc_i2c_context;
static const cy_stc_scb_i2c_config_t  g_stc_i2c_config =
{
    .i2cMode             = CY_SCB_I2C_MASTER,
    .useRxFifo           = true,
    .useTxFifo           = true,
    .slaveAddress        = AMP_I2C_SLAVE_ADDR,
    .slaveAddressMask    = AMP_I2C_SLAVE_ADDR,
    .acceptAddrInFifo    = false,
    .ackGeneralAddr      = false,
    .enableWakeFromSleep = false
};

static cy_stc_scb_i2c_master_xfer_config_t g_stc_i2c_master_config =
{
    .slaveAddress = AMP_I2C_SLAVE_ADDR,
    .buffer       = 0,
    .bufferSize   = 0,
    .xferPending  = false
};
#endif //ESTEC_I2C_ENABLE

//Function Declaration ***********************************************************************/
#ifdef ESTEC_I2C_ENABLE
void Scb_I2C_IntrISR(void);
void SetI2CPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum);

uint8_t Scb_I2C_Master_Write(uint8_t reg_addr, uint8_t reg_data);
uint8_t Scb_I2C_Master_Read(uint8_t reg_addr, uint8_t *reg_data);
#endif

//Function Definition ***********************************************************************/
#ifdef ESTEC_I2C_ENABLE
void Scb_I2C_IntrISR(void)
{
    /* I2C interrupt handler for High-Level APIs */
    Cy_SCB_I2C_Interrupt(USER_I2C_SCB_TYPE, &g_stc_i2c_context);
}

uint8_t Scb_I2C_Master_Write(uint8_t reg_addr, uint8_t reg_data)
{
    cy_en_scb_i2c_status_t status;

	// Write Start condition + Slave Address
    status = Cy_SCB_I2C_MasterSendStart(USER_I2C_SCB_TYPE, AMP_I2C_SLAVE_ADDR, CY_SCB_I2C_WRITE_XFER, 2000ul, &g_stc_i2c_context);

	if(status == CY_SCB_I2C_SUCCESS)
	{	// Write Sub-Address
    	status = Cy_SCB_I2C_MasterWriteByte(USER_I2C_SCB_TYPE, reg_addr, 2000ul, &g_stc_i2c_context);

		//Write Data
	    if(status == CY_SCB_I2C_SUCCESS)
			status = Cy_SCB_I2C_MasterWriteByte(USER_I2C_SCB_TYPE, reg_data, 2000ul, &g_stc_i2c_context);
	}

    Cy_SCB_I2C_MasterSendStop(USER_I2C_SCB_TYPE, 2000ul, &g_stc_i2c_context);

	return status;
}

uint8_t Scb_I2C_Master_Read(uint8_t reg_addr, uint8_t *reg_data)
{
    cy_en_scb_i2c_status_t status;

    // Write Start condition + Slave Address
    status = Cy_SCB_I2C_MasterSendStart(USER_I2C_SCB_TYPE, AMP_I2C_SLAVE_ADDR, CY_SCB_I2C_WRITE_XFER, 2000ul, &g_stc_i2c_context);
	
    if(status == CY_SCB_I2C_SUCCESS) 
    {
		// Write Sub-Address
	    status = Cy_SCB_I2C_MasterWriteByte(USER_I2C_SCB_TYPE, reg_addr, 2000ul, &g_stc_i2c_context);

		if(status == CY_SCB_I2C_SUCCESS)
	    {
	    	// Repeated start for Read
	    	status = Cy_SCB_I2C_MasterSendReStart(USER_I2C_SCB_TYPE, AMP_I2C_SLAVE_ADDR, CY_SCB_I2C_READ_XFER, 2000ul, &g_stc_i2c_context);

			//Read Data
			if(status == CY_SCB_I2C_SUCCESS)
			    Cy_SCB_I2C_MasterReadByte(USER_I2C_SCB_TYPE, CY_SCB_I2C_NAK, reg_data, 2000ul, &g_stc_i2c_context);
	    }
    }

    Cy_SCB_I2C_MasterSendStop(USER_I2C_SCB_TYPE, 2000ul, &g_stc_i2c_context);

    return status;
}

void SetI2CPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
    uint64_t temp = ((uint64_t)sourceFreq << 5ull);
    uint32_t divSetting;

    divSetting = (uint32_t)(temp / targetFreq);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(USER_I2C_SCB_PCLK), 
                                   CY_SYSCLK_DIV_24_5_BIT, divNum, 
                                   (((divSetting >> 5u) & 0x00000FFFul) - 1ul), 
                                   (divSetting & 0x0000001Ful));
}

#endif //ESTEC_I2C_ENABLE

void Fda803d_AmpInit(void)
{
  uint8_t data = 0;
	uint8_t result = 0;

#ifdef ESTEC_I2C_ENABLE
	/*---------------------*/
	/* Clock Configuration */
	/*---------------------*/
	Cy_SysClk_PeriphAssignDivider(USER_I2C_SCB_PCLK, CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
	SetI2CPeripheFracDiv24_5(E_I2C_INCLK_TARGET_FREQ, E_SOURCE_CLK_FREQ, DIVIDER_NO_1);
	Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(USER_I2C_SCB_PCLK), CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);

	/*--------------------*/
	/* Port Configuration */
	/*--------------------*/
	i2c_port_pin_cfg.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
	i2c_port_pin_cfg.hsiom	   = I2C_SDA_PORT_MUX;
	Cy_GPIO_Pin_Init(I2C_SDA_PORT, I2C_SDA_PORT_PIN, &i2c_port_pin_cfg);

	i2c_port_pin_cfg.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
	i2c_port_pin_cfg.hsiom	   = I2C_SCL_PORT_MUX;
	Cy_GPIO_Pin_Init(I2C_SCL_PORT, I2C_SCL_PORT_PIN, &i2c_port_pin_cfg);

#ifdef ESTEC_GPIO_ENABLE
	Cy_GPIO_Pin_Init(AMP_MASTER_ON_PORT, AMP_MASTER_ON_PIN, &user_amp_on_port_pin_cfg);
	Cy_GPIO_Pin_Init(AMP_MASTER_MUTE_PORT, AMP_MASTER_MUTE_PIN, &user_amp_on_port_pin_cfg);
#endif
	/*--------------------------*/
	/* Interrupt Configuration */
	/*--------------------------*/
	Cy_SysInt_InitIRQ(&i2c_irq_cfg);
	Cy_SysInt_SetSystemIrqVector(i2c_irq_cfg.sysIntSrc, Scb_I2C_IntrISR);
	NVIC_SetPriority(i2c_irq_cfg.intIdx, 3ul);
	NVIC_EnableIRQ(i2c_irq_cfg.intIdx);

	/*--------------------------*/
	/*	Initialize & Enable I2C  */
	/*--------------------------*/
	Cy_SCB_I2C_DeInit(USER_I2C_SCB_TYPE);
	Cy_SCB_I2C_Init(USER_I2C_SCB_TYPE, &g_stc_i2c_config, &g_stc_i2c_context);
	Cy_SCB_I2C_SetDataRate(USER_I2C_SCB_TYPE, E_I2C_DATARATE, E_I2C_INCLK_TARGET_FREQ);
	Cy_SCB_I2C_RegisterEventCallback(USER_I2C_SCB_TYPE, NULL, &g_stc_i2c_context);
	Cy_SCB_I2C_Enable(USER_I2C_SCB_TYPE);	
#endif

#ifdef ESTEC_GPIO_ENABLE
	Cy_GPIO_Clr(AMP_MASTER_MUTE_PORT, AMP_MASTER_MUTE_PIN); //FDA803D HW mute
	Cy_GPIO_Set(AMP_MASTER_ON_PORT, AMP_MASTER_ON_PIN); //FDA803D Enalbe2 high -> i2c address : 0x70 , Stand by -> Diag Vcc/Gnd, 90ms ?´í›„ -> ECO mode
#endif

    Cy_SysTick_DelayInUs(200000); //200ms wait - 90ms short check + 90ms Stable condition

    result = Scb_I2C_Master_Read(FDA803D_DB2_REG, &data);

    result = Scb_I2C_Master_Read(FDA803D_IB1_REG, &data);
    data |= IB1_WS_FREQUENCY;
    result = Scb_I2C_Master_Write(FDA803D_IB1_REG, data);

    result = Scb_I2C_Master_Read(FDA803D_IB2_REG, &data);
    data |= IB2_FULL_SCALE_VOLTAGE_LIMIT_80;
    result = Scb_I2C_Master_Write(FDA803D_IB2_REG, data);

    result = Scb_I2C_Master_Read(FDA803D_IB8_REG, &data);
    data |= IB8_PWM_ON | IB8_CHANNEL_IN_MUTE;
    result = Scb_I2C_Master_Write(FDA803D_IB8_REG, data); //pwm off, channel in mute
    
    result = Scb_I2C_Master_Read(FDA803D_IB1_REG, &data);
    result = Scb_I2C_Master_Read(FDA803D_IB2_REG, &data);
    result = Scb_I2C_Master_Read(FDA803D_IB8_REG, &data); //pwm off, channel in mute
#ifdef ESTEC_GPIO_ENABLE
	Cy_GPIO_Set(AMP_MASTER_MUTE_PORT, AMP_MASTER_MUTE_PIN); //FDA803D HW unmute
#endif
}

#endif //FDA806D_AMP_ENABLE

