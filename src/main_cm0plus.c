/***************************************************************************//**
* \file main_cm7_0.c
*
* \version 1.0
*
* \brief Main example file for CM7_0
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

//#define NON_ISO_OPERATION 1 //KMS250908
#define ESTEC_CAN_ENABLE			(1)
#ifdef ESTEC_CAN_ENABLE
#define ESTEC_SPI_ENABLE_FOR_CAN	(1)
#endif

#define FDA806D_AMP_ENABLE			(1)
#ifdef FDA806D_AMP_ENABLE
#define ESTEC_I2C_ENABLE			(1)
#define ESTEC_GPIO_ENABLE			(1)
#endif

//MACRO & Define ***********************************************************************/
#ifdef ESTEC_CAN_ENABLE
/* CAN in Use */
#define CY_CANFD_TYPE                   CY_CANFD1_TYPE   
#define CY_CANFD_RX_PORT                CY_CANFD1_RX_PORT
#define CY_CANFD_RX_PIN                 CY_CANFD1_RX_PIN 
#define CY_CANFD_TX_PORT                CY_CANFD1_TX_PORT
#define CY_CANFD_TX_PIN                 CY_CANFD1_TX_PIN 
#define CY_CANFD_RX_MUX                 CY_CANFD1_RX_MUX 
#define CY_CANFD_TX_MUX                 CY_CANFD1_TX_MUX 
#define CY_CANFD_PCLK                   CY_CANFD1_PCLK
#define CY_CANFD_IRQN                   CY_CANFD1_IRQN
#endif //ESTEC_CAN_ENABLE

//SPI Setting
#ifdef ESTEC_SPI_ENABLE_FOR_CAN
/* Master Settings */
#define SCB_MISO_DRIVE_MODE 			CY_GPIO_DM_HIGHZ
#define SCB_MOSI_DRIVE_MODE 			CY_GPIO_DM_STRONG_IN_OFF
#define SCB_CLK_DRIVE_MODE  			CY_GPIO_DM_STRONG_IN_OFF
#define SCB_SEL0_DRIVE_MODE 			CY_GPIO_DM_STRONG_IN_OFF

#define SCB_SPI_BAUDRATE     125000ul /* Please set baudrate value of SPI you want */
#define SCB_SPI_OVERSAMPLING 16ul     /* Please set oversampling of SPI you want */
#define SCB_SPI_CLOCK_FREQ (SCB_SPI_BAUDRATE * SCB_SPI_OVERSAMPLING)

#define DIVIDER_NO_1 (1u)
#endif //ESTEC_SPI_ENABLE_FOR_CAN

/* User setting value */
#if (CY_USE_PSVP == 1)  
  #define SOURCE_CLOCK_FRQ 24000000ul
  #define CORE_CLOCK_FRQ   24000000ul
#else
  #define SOURCE_CLOCK_FRQ 80000000ul
  #define CORE_CLOCK_FRQ   80000000ul
#endif

#ifdef ESTEC_CAN_ENABLE
//#define TJA_1146_CAN_SUPPORT		(1)
#define TACN_1146_CAN_SUPPORT		(1)

#ifdef TJA1145_CAN_SUPPORT
#define WRITE_REG							(0x00u)
#define MODE_CONTROL_REG_ADDR				(0x01u)
#define MODE_CONTROL_NORMAL_MODE			(0x07u)
#else //TACN1145_CAN_SUPPORT
#define WRITE_REG							(0x01u)
#define MODE_CONTROL_REG_ADDR				(0x10u)
#define MODE_CONTROL_NORMAL_MODE			(0x07u)
#define MODE_CONTROL_STANDBY_MODE			(0x04u)
#endif
#endif

//Function & Declaration & definition ***********************************************************************/
//CAN
static void PortInit(void);

#ifdef ESTEC_CAN_ENABLE
void CanfdInterruptHandler(void);

void CAN_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg);
void CAN_RxFifoWithTopCallback(uint8_t u8FifoNum, uint8_t u8BufferSizeInWord, uint32_t* pu32RxBuf);

#if NON_ISO_OPERATION == 1
static void SetISOFormat(cy_pstc_canfd_type_t canfd);
#endif

cy_stc_canfd_msg_t CanMsg;

/* Port configuration */
typedef struct
{
    volatile stc_GPIO_PRT_t* portReg;
    uint8_t pinNum;
    cy_stc_gpio_pin_config_t cfg;
}stc_pin_config;


/* Standard ID Filter configration */
static const cy_stc_id_filter_t stdIdFilter[] = 
{
    CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(0x010u, 0u),      /* ID=0x010, store into RX buffer Idx0 */
    CANFD_CONFIG_STD_ID_FILTER_CLASSIC_RXBUFF(0x020u, 1u),      /* ID=0x020, store into RX buffer Idx1 */
};

/* Extended ID Filter configration */
static const cy_stc_extid_filter_t extIdFilter[] = 
{
    CANFD_CONFIG_EXT_ID_FILTER_CLASSIC_RXBUFF(0x10010u, 2u),    /* ID=0x10010, store into RX buffer Idx2 */
    CANFD_CONFIG_EXT_ID_FILTER_CLASSIC_RXBUFF(0x10020u, 3u),    /* ID=0x10020, store into RX buffer Idx3 */
};

/* CAN configuration */
cy_stc_canfd_config_t canCfg = 
{
    .txCallback     = NULL, // Unused.
    .rxCallback     = CAN_RxMsgCallback,
    .rxFifoWithTopCallback = NULL, //CAN_RxFifoWithTopCallback,
    .statusCallback = NULL, // Un-supported now
    .errorCallback  = NULL, // Un-supported now

    .canFDMode      = true, // Use CANFD mode
  #if (CY_USE_PSVP == 1)
    // 24 MHz
    .bitrate        =       // Nominal bit rate settings (sampling point = 75%)
    {
        .prescaler      = 6u - 1u,  // cclk/6, When using 500kbps, 1bit = 8tq
        .timeSegment1   = 5u - 1u,  // tseg1 = 5tq
        .timeSegment2   = 2u - 1u,  // tseg2 = 2tq
        .syncJumpWidth  = 2u - 1u,  // sjw   = 2tq
    },
    
    .fastBitrate    =       // Fast bit rate settings (sampling point = 75%)
    {
        .prescaler      = 3u - 1u,  // cclk/3, When using 1Mbps, 1bit = 8tq
        .timeSegment1   = 5u - 1u,  // tseg1 = 5tq,
        .timeSegment2   = 2u - 1u,  // tseg2 = 2tq
        .syncJumpWidth  = 2u - 1u,  // sjw   = 2tq
    },
  #else
    // 40 MHz
    .bitrate        =       // Nominal bit rate settings (sampling point = 75%)
    {
        .prescaler      = 10u - 1u,  // cclk/10, When using 500kbps, 1bit = 8tq
        .timeSegment1   = 5u - 1u,  // tseg1 = 5tq
        .timeSegment2   = 2u - 1u,  // tseg2 = 2tq
        .syncJumpWidth  = 2u - 1u,  // sjw   = 2tq
    },
    
    .fastBitrate    =       // Fast bit rate settings (sampling point = 75%)
    {
        .prescaler      = 5u - 1u,  // cclk/5, When using 1Mbps, 1bit = 8tq
        .timeSegment1   = 5u - 1u,  // tseg1 = 5tq,
        .timeSegment2   = 2u - 1u,  // tseg2 = 2tq
        .syncJumpWidth  = 2u - 1u,  // sjw   = 2tq
    },
  #endif      
    .tdcConfig      =       // Transceiver delay compensation, unused.
    {
        .tdcEnabled     = false,
        .tdcOffset      = 0,
        .tdcFilterWindow= 0,
    },
    .sidFilterConfig    =   // Standard ID filter
    {
        .numberOfSIDFilters = sizeof(stdIdFilter) / sizeof(stdIdFilter[0]),
        .sidFilter          = stdIdFilter,
    },
    .extidFilterConfig  =   // Extended ID filter
    {
        .numberOfEXTIDFilters   = sizeof(extIdFilter) / sizeof(extIdFilter[0]),
        .extidFilter            = extIdFilter,
        .extIDANDMask           = 0x1fffffff,   // No pre filtering.
    },
    .globalFilterConfig =   // Global filter
    {
        .nonMatchingFramesStandard = CY_CANFD_ACCEPT_IN_RXFIFO_0,  // Reject none match IDs
        .nonMatchingFramesExtended = CY_CANFD_ACCEPT_IN_RXFIFO_1,  // Reject none match IDs
        .rejectRemoteFramesStandard = true, // No remote frame
        .rejectRemoteFramesExtended = true, // No remote frame
    },
    .rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_64,
    .rxFifo1DataSize  = CY_CANFD_BUFFER_DATA_SIZE_64,
    .rxFifo0DataSize  = CY_CANFD_BUFFER_DATA_SIZE_64,
    .txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_64,
    .rxFifo0Config    = // RX FIFO0, unused.
    {
        .mode = CY_CANFD_FIFO_MODE_BLOCKING,
        .watermark = 10u,
        .numberOfFifoElements = 8u,
        .topPointerLogicEnabled = false,
    },
    .rxFifo1Config    = // RX FIFO1, unused.
    {
        .mode = CY_CANFD_FIFO_MODE_BLOCKING,
        .watermark = 10u,
        .numberOfFifoElements = 8u,
        .topPointerLogicEnabled = false, // true,
    },
    .noOfRxBuffers  = 4u,
    .noOfTxBuffers  = 4u,
};

/* CAN port configuration */
static const stc_pin_config can_pin_cfg[] =
{
    /* CAN0_2 RX */
    {
        .portReg = CY_CANFD_RX_PORT, 
        .pinNum  = CY_CANFD_RX_PIN,
        {
            .outVal = 0,
            .driveMode = CY_GPIO_DM_HIGHZ,
            .hsiom = CY_CANFD_RX_MUX,
            .intEdge = 0,
            .intMask = 0,
            .vtrip = 0,
            .slewRate = 0,
            .driveSel = 0,
            .vregEn = 0,
            .ibufMode = 0,
            .vtripSel = 0,
            .vrefSel = 0,
            .vohSel = 0,
        }
    },
    /* CAN0_2 TX */
    {
        .portReg = CY_CANFD_TX_PORT,
        .pinNum  = CY_CANFD_TX_PIN,
        {
            .outVal = 1,
            .driveMode = CY_GPIO_DM_STRONG,
            .hsiom = CY_CANFD_TX_MUX,
            .intEdge = 0,
            .intMask = 0,
            .vtrip = 0,
            .slewRate = 0,
            .driveSel = 0,
            .vregEn = 0,
            .ibufMode = 0,
            .vtripSel = 0,
            .vrefSel = 0,
            .vohSel = 0,
        }
    },
};
#endif //ESTEC_CAN_ENABLE

//SPI
#ifdef ESTEC_SPI_ENABLE_FOR_CAN
void SetSPIPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum);
void Spi_Init(void);

#ifdef SPI_TEST
void Spi_Send_TCAN1445_Power_On(void); //KMS250904_1
void Spi_Send_TCAN1445_Power_Off(void); //KMS250904_1
#endif

static cy_stc_gpio_pin_config_t SPI_port_pin_cfg =
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

static cy_stc_sysint_irq_t irq_cfg_spi =
{
    .sysIntSrc  = CY_SPI_SCB_IRQN,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

static const cy_stc_scb_spi_config_t SCB_SPI_cfg =
{
    .spiMode                    = CY_SCB_SPI_MASTER,      /*** Specifies the mode of operation    ***/
    .subMode                    = CY_SCB_SPI_MOTOROLA,    /*** Specifies the sub mode of SPI operation    ***/
    .sclkMode                   = CY_SCB_SPI_CPHA0_CPOL0, /*** Clock is active low, data is changed on first edge ***/
    .oversample                 = SCB_SPI_OVERSAMPLING,   /*** SPI_CLOCK divided by SCB_SPI_OVERSAMPLING should be baudrate  ***/
    .rxDataWidth                = 16ul,                   /*** The width of RX data (valid range 4-16). It must be the same as \ref txDataWidth except in National sub-mode. ***/
    .txDataWidth                = 16ul,                   /*** The width of TX data (valid range 4-16). It must be the same as \ref rxDataWidth except in National sub-mode. ***/
    .enableMsbFirst             = true,                   /*** Enables the hardware to shift out the data element MSB first, otherwise, LSB first ***/
    .enableFreeRunSclk          = false,                  /*** Enables the master to generate a continuous SCLK regardless of whether there is data to send  ***/
    .enableInputFilter          = false,                  /*** Enables a digital 3-tap median filter to be applied to the input of the RX FIFO to filter glitches on the line. ***/
    .enableMisoLateSample       = true,                   /*** Enables the master to sample MISO line one half clock later to allow better timings. ***/
    .enableTransferSeperation   = true,                   /*** Enables the master to transmit each data element separated by a de-assertion of the slave select line (only applicable for the master mode) ***/
    .ssPolarity0                = false,                  /*** SS0: active low ***/
    .ssPolarity1                = false,                  /*** SS1: active low ***/
    .ssPolarity2                = false,                  /*** SS2: active low ***/
    .ssPolarity3                = false,                  /*** SS3: active low ***/
    .enableWakeFromSleep        = false,                  /*** When set, the slave will wake the device when the slave select line becomes active. Note that not all SCBs support this mode. Consult the device datasheet to determine which SCBs support wake from deep sleep. ***/
    .rxFifoTriggerLevel         = 1ul,                    /*** Interrupt occurs, when there are more entries of 2 in the RX FIFO */
    .rxFifoIntEnableMask        = 1ul,                    /*** Bits set in this mask will allow events to cause an interrupt  */
    .txFifoTriggerLevel         = 0ul,                    /*** When there are fewer entries in the TX FIFO, then at this level the TX trigger output goes high. This output can be connected to a DMA channel through a trigger mux. Also, it controls the \ref CY_SCB_SPI_TX_TRIGGER interrupt source. */
    .txFifoIntEnableMask        = 0ul,                    /*** Bits set in this mask allow events to cause an interrupt  */
    .masterSlaveIntEnableMask   = 0ul,                    /*** Bits set in this mask allow events to cause an interrupt  */
    .enableSpiDoneInterrupt     = false,
    .enableSpiBusErrorInterrupt = false,
};

typedef enum {
	CAN_INITIAL_MODE = (0u),
	CAN_NORMAL_MODE,
	CAN_STANDBY_MODE
} CAN_MODE;

#define SPI_READ_BIT            	(0u)
#define SPI_WRITE_BIT            	(1u)

#define CAN_MODE_CONTROL_REG						(0x10)
#define CAN_MODE_CONTROL_REG_SET_STANDBY			(0x01)
#define CAN_MODE_CONTROL_REG_SET_NORMAL				(0x07)


CAN_MODE current_can_mode = CAN_STANDBY_MODE;

uint16_t readData[2], writeData[2];
volatile unsigned int uIntterupt_check = 0; 

#endif //ESTEC_SPI_ENABLE_FOR_CAN


/* Select Frequency */
#if (CY_USE_PSVP == 1)
  #define E_SOURCE_CLK_FREQ     (24000000ul) // fixed
#else
  #define E_SOURCE_CLK_FREQ     (80000000u)  // fixed
#endif

#ifdef ESTEC_I2C_ENABLE
#define E_I2C_INCLK_TARGET_FREQ (2000000ul)  // modifiable
#define E_I2C_DATARATE          (100000ul)   // modifiable
#endif

#ifdef FDA806D_AMP_ENABLE
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

//#define FDA803D_IB14_REG              	0x8e

#define FDA803D_DB2_REG               		0xA2
#define DB2_CHANNEL_IN_MUTE           		0x00
#define DB2_CHANNEL_IN_PLAY           		0x01

#ifdef ESTEC_I2C_ENABLE
#define USER_I2C_SCB_TYPE       SCB2
#define USER_I2C_SCB_PCLK       PCLK_SCB2_CLOCK
#define USER_I2C_SCB_IRQN       scb_2_interrupt_IRQn

#define I2C_SDA_PORT     		GPIO_PRT19
#define I2C_SDA_PORT_PIN 		(1)
#define I2C_SDA_PORT_MUX 		P19_1_SCB2_I2C_SDA

#define I2C_SCL_PORT     		GPIO_PRT19
#define I2C_SCL_PORT_PIN 		(2)
#define I2C_SCL_PORT_MUX 		P19_2_SCB2_I2C_SCL


void SetI2CPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum);

void Scb_I2C_IntrISR(void);
uint8_t Scb_I2C_Master_Write(uint8_t reg_addr, uint8_t reg_data);
uint8_t Scb_I2C_Master_Read(uint8_t reg_addr, uint8_t *reg_data);

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
void Fda803d_AmpInit(void);
#endif //FDA806D_AMP_ENABLE


/* Local Variables */

#ifdef ESTEC_CAN_ENABLE
//CAN Function
void CAN_Send_Test(void)
{
	/* Prepare CANFD message to transmit*/
    cy_stc_canfd_msg_t stcMsg;
	
    Cy_SysLib_Delay(1000);
       
    stcMsg.canFDFormat = true;
    stcMsg.idConfig.extended = false;
    stcMsg.idConfig.identifier = 0x525;
    stcMsg.dataConfig.dataLengthCode = 15;
    stcMsg.dataConfig.data[0]  = 0x70190523;
    stcMsg.dataConfig.data[1]  = 0x70190819;
    stcMsg.dataConfig.data[2]  = 0x33332222;
    stcMsg.dataConfig.data[3]  = 0x33332222;
    stcMsg.dataConfig.data[4]  = 0x55554444;
    stcMsg.dataConfig.data[5]  = 0x77776666;
    stcMsg.dataConfig.data[6]  = 0x99998888;
    stcMsg.dataConfig.data[7]  = 0xBBBBAAAA;
    stcMsg.dataConfig.data[8]  = 0xDDDDCCCC;
    stcMsg.dataConfig.data[9]  = 0xFFFFEEEE;
    stcMsg.dataConfig.data[10] = 0x78563412;
    stcMsg.dataConfig.data[11] = 0x00000000;
    stcMsg.dataConfig.data[12] = 0x11111111;
    stcMsg.dataConfig.data[13] = 0x22222222;
    stcMsg.dataConfig.data[14] = 0x33333333;
    stcMsg.dataConfig.data[15] = 0x44444444;

    Cy_CANFD_UpdateAndTransmitMsgBuffer(CY_CANFD_TYPE, 0, &stcMsg);
}

void CAN_Init1(void)
{
    /* Setup CAN clock (cclk).
     * This clock is used as base clock of the CAN communication.
     */
    {
        Cy_SysClk_HfClkEnable(CY_SYSCLK_HFCLK_2);
        
        /* PSVP: In this example, no divid, just enable the clock.-> Use 24MHz (PSVP default PERI clock) as cclk.
           Silicon: Use divider 2 --> 40MHz
         */
        Cy_SysClk_PeriphAssignDivider(CY_CANFD_PCLK, CY_SYSCLK_DIV_8_BIT, 0u);
      #if (CY_USE_PSVP == 1)
        Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(CY_CANFD_PCLK), CY_SYSCLK_DIV_8_BIT, 0u, 0u); // 24 MHz
      #else
        Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(CY_CANFD_PCLK), CY_SYSCLK_DIV_8_BIT, 0u, 1u); // 40 MHz
      #endif
        Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(CY_CANFD_PCLK), CY_SYSCLK_DIV_8_BIT, 0u);
    }

    /* For PSVP, DeInit to initialize CANFD IP */
    {
        Cy_CANFD_DeInit(CY_CANFD_TYPE);
    }
}

void CAN_Init2(void)
{
    /* Setup CANFD interrupt */
    cy_stc_sysint_irq_t irq_cfg;
    irq_cfg = (cy_stc_sysint_irq_t){
    .sysIntSrc  = CY_CANFD_IRQN, /* Use interrupt LINE0 */
    .intIdx     = CPUIntIdx2_IRQn,
    .isEnabled  = true,
    };
    
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, CanfdInterruptHandler);

    NVIC_SetPriority(CPUIntIdx2_IRQn, 0);

    NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);
    NVIC_EnableIRQ(CPUIntIdx2_IRQn);

    /* Initialize CAN as CANFD */
    Cy_CANFD_Init(CY_CANFD_TYPE, &canCfg);
#if NON_ISO_OPERATION == 1
    SetISOFormat(CY_CANFD_TYPE);
#endif
    /* Now a ch configured as CANFD is working. */
}    
#endif //ESTEC_CAN_ENABLE

#ifdef ESTEC_SPI_ENABLE_FOR_CAN
uint8_t SPI_COM_CANTransceiver_Read(uint8_t address)
{
    uint8_t addr = 0;
    uint8_t getTrcvStatus =0x0;
    
    //Command Read
    uIntterupt_check = 0;
    writeData[0] = 0;
    addr= (address << 0x1) | SPI_READ_BIT; //read address
    writeData[0] = addr << 8;
	Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1); 

	while(!uIntterupt_check)
	{		
		Cy_SysLib_Delay(100); //To avoid too many calls
		Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1); //Send again
	}
		
    getTrcvStatus = readData[0] & 0x00FF;

    return getTrcvStatus;
}

void SPI_COM_CANTransceiver_Write(uint8_t address, uint8_t mode)
{
    uint8_t addr = 0;
    uint8_t command = 0;
    uint8_t requestTrcvStatus = 0;
    uint8_t getTrcvStatus = 0;
	
    //Command Write
    writeData[0] = 0;
    addr= (address << 0x1) | SPI_WRITE_BIT; //write address
    command= mode;
    writeData[0] = (addr << 8) | command ;
    requestTrcvStatus = writeData[0] & 0x00FF; 
   
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1); 
		
    while(1)
    {
        //Command Read
        writeData[0] = 0;
		addr= (address << 0x1) | SPI_READ_BIT; //read address
		writeData[0] = (addr << 8);
        Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1); 

        getTrcvStatus = readData[0] & 0x00FF; 

        if(getTrcvStatus == requestTrcvStatus)
        {
           break;
        }
		else
			Cy_SysLib_Delay(100); //To avoid too many calls
    }
}

void CAN_Transceiver_Status(CAN_MODE cur_mode)
{
    uint8_t uGet_Status;
    
    switch(cur_mode)
    {
    	case CAN_STANDBY_MODE:
            //SPI_COM_CANTransceiver_Write(CAN_MODE_CONTROL_REG, CAN_MODE_CONTROL_REG_SET_STANDBY);
            current_can_mode = CAN_INITIAL_MODE;
            break;
		
        case CAN_INITIAL_MODE:
            SPI_COM_CANTransceiver_Write(CAN_MODE_CONTROL_REG, CAN_MODE_CONTROL_REG_SET_NORMAL);
            current_can_mode = CAN_NORMAL_MODE;
            break;

        case CAN_NORMAL_MODE:
            uGet_Status = SPI_COM_CANTransceiver_Read(CAN_MODE_CONTROL_REG);
            
            if(uGet_Status == 0x7)
            {
                CanMsg.canFDFormat = true;
                CanMsg.idConfig.extended = false;
                CanMsg.idConfig.identifier = 0x525;
                CanMsg.dataConfig.dataLengthCode = 15;
                CanMsg.dataConfig.data[0] = 0x70190523;
                CanMsg.dataConfig.data[1] = 0x70190819;
                CanMsg.dataConfig.data[2] = 0x33332222;
                CanMsg.dataConfig.data[3] = 0x33332222;
                CanMsg.dataConfig.data[4] = 0x55554444;
                CanMsg.dataConfig.data[5] = 0x77776666;
                CanMsg.dataConfig.data[6] = 0x99998888;
                CanMsg.dataConfig.data[7] = 0xBBBBAAAA;
                CanMsg.dataConfig.data[8] = 0xDDDDCCCC;
                CanMsg.dataConfig.data[9] = 0xFFFFEEEE;
                CanMsg.dataConfig.data[10] = 0x78563412;
                CanMsg.dataConfig.data[11] = 0x00000000;
                CanMsg.dataConfig.data[12] = 0x11111111;
                CanMsg.dataConfig.data[13] = 0x22222222;
                CanMsg.dataConfig.data[14] = 0x33333333;
                CanMsg.dataConfig.data[15] = 0x44444444;
                
                Cy_CANFD_UpdateAndTransmitMsgBuffer(CY_CANFD_TYPE, 0, &CanMsg);
            }
                                                                    
            break;
    }  
}
                                  
//SPI Function
void irqSCB(void)
{
    uint32_t status;

    status = Cy_SCB_SPI_GetRxFifoStatus(CY_SPI_SCB_TYPE);
    if(status & CY_SCB_SPI_RX_TRIGGER)
    {
        uIntterupt_check = 1;
        Cy_SCB_SPI_ReadArray(CY_SPI_SCB_TYPE, (void*)readData, 2);
        Cy_SCB_SPI_ClearRxFifoStatus(CY_SPI_SCB_TYPE, CY_SCB_SPI_RX_TRIGGER);
    }
}

void SetSPIPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
    uint64_t temp = ((uint64_t)sourceFreq << 5ull);
    uint32_t divSetting;

    divSetting = (uint32_t)(temp / targetFreq);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(CY_SPI_SCB_PCLK), 
                                   CY_SYSCLK_DIV_24_5_BIT, divNum, 
                                   (((divSetting >> 5ul) & 0x00000FFFul) - 1ul), 
                                   (divSetting & 0x0000001Ful));
}

#ifdef SPI_TEST
void Spi_Send_TCAN1445_Power_On(void) //KMS050904_1
{    
	writeData[0] = 0x8950;
	Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);

    writeData[0] = 0xA540; //INT2 - VIO under voltage interrupt
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA400; //INT2 - VIO under voltage interrupt
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);

    writeData[0] = 0xA300; //INT1
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA200; //INT1
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA500; //INT2 - VIO under voltage interrupt --> Claer
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA400; //INT2 - VIO under voltage interrupt --> Claer
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);

    writeData[0] = 0x2304; //WAKE-Pin config
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2200; //WAKE-Pin config
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2F01; //FSM(Fail Safety Mode) config --> No configuration
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2E00; //FSM(Fail Safety Mode) config --> No configuration
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);

    writeData[0] = 0x3984; //SWE_DIS config --> Sleep wake error timer on
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x3800; //SWE_DIS config --> Sleep wake error timer on
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA300; //INT1
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA200; //INT1
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);

    writeData[0] = 0x2107; // Mode Control --> Normal Mode
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2000; // Mode Control --> Normal Mode
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);

    writeData[0] = 0x2304; //WAKE-Pin config
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2204; //WAKE-Pin config
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
}

void Spi_Send_TCAN1445_Power_Off(void) //KMS050904_1
{
    //TBD : Need to deleted unaffected codes.
    writeData[0] = 0x8950;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA540;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA304;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA500;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2304;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2F01;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x3984;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA300;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2107;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2304;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2107;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2304;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA540;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0xA300;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2304;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
    
    writeData[0] = 0x2101;
    Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)writeData, 1);
}
#endif

void Spi_Init(void)
{
    /******************************************************/
    /******* Calculate divider setting for the SCB ********/
    /******************************************************/
    Cy_SysClk_PeriphAssignDivider(CY_SPI_SCB_PCLK, CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
    SetSPIPeripheFracDiv24_5(SCB_SPI_CLOCK_FREQ, SOURCE_CLOCK_FRQ, DIVIDER_NO_1);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(CY_SPI_SCB_PCLK), CY_SYSCLK_DIV_24_5_BIT, 1u);
    
    /********************************************/
    /*    De-initialization for peripherals     */
    /********************************************/
    Cy_SCB_SPI_DeInit(CY_SPI_SCB_TYPE);

    /*******************************************/
    /* Interrupt setting for SPI communication */
    /*******************************************/
    Cy_SysInt_InitIRQ(&irq_cfg_spi);
    Cy_SysInt_SetSystemIrqVector(irq_cfg_spi.sysIntSrc, irqSCB);
    NVIC_EnableIRQ(irq_cfg_spi.intIdx);

    /**************************************/
    /* Port Setting for SPI communication */
    /**************************************/
    /* According to the HW environment to change SCB CH*/
    SPI_port_pin_cfg.driveMode = SCB_MISO_DRIVE_MODE;
    SPI_port_pin_cfg.hsiom = CY_SPI_SCB_MISO_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_MISO_PORT, CY_SPI_SCB_MISO_PIN, &SPI_port_pin_cfg);

    SPI_port_pin_cfg.driveMode = SCB_MOSI_DRIVE_MODE;
    SPI_port_pin_cfg.hsiom = CY_SPI_SCB_MOSI_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_MOSI_PORT, CY_SPI_SCB_MOSI_PIN, &SPI_port_pin_cfg);

    SPI_port_pin_cfg.driveMode = SCB_CLK_DRIVE_MODE;
    SPI_port_pin_cfg.hsiom = CY_SPI_SCB_CLK_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_CLK_PORT,CY_SPI_SCB_CLK_PIN, &SPI_port_pin_cfg);
    
    SPI_port_pin_cfg.driveMode = SCB_SEL0_DRIVE_MODE;
    SPI_port_pin_cfg.hsiom = CY_SPI_SCB_SEL0_MUX;
    Cy_GPIO_Pin_Init(CY_SPI_SCB_SEL0_PORT, CY_SPI_SCB_SEL0_PIN, &SPI_port_pin_cfg);

    /********************************************/
    /* SCB initialization for SPI communication */
    /********************************************/
    Cy_SCB_SPI_Init(CY_SPI_SCB_TYPE, &SCB_SPI_cfg, NULL);
    Cy_SCB_SPI_SetActiveSlaveSelect(CY_SPI_SCB_TYPE, 0ul);
    Cy_SCB_SPI_Enable(CY_SPI_SCB_TYPE);

    /********************************************/
    /*      Write initial value to buffer       */
    /********************************************/
    //SchedulerInit();
    //Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE,(void*)readData, 2);

    //Normal Mode Setting
    //Spi_Send_TCAN1445_Power_On();
}
#endif //ESTEC_SPI_ENABLE_FOR_CAN

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
    CAN_Init1(); //CAN Init1

	/* Initialize CAN ports and the CAN tranceiver. */
    {
        PortInit();
    }
	
    CAN_Init2(); //CAN Init2

    //current_can_mode = CAN_INITIAL_MODE;
#endif
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
    i2c_port_pin_cfg.hsiom     = I2C_SDA_PORT_MUX;
    Cy_GPIO_Pin_Init(I2C_SDA_PORT, I2C_SDA_PORT_PIN, &i2c_port_pin_cfg);

    i2c_port_pin_cfg.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
    i2c_port_pin_cfg.hsiom     = I2C_SCL_PORT_MUX;
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
    /*  Initialize & Enable I2C  */
    /*--------------------------*/
    Cy_SCB_I2C_DeInit(USER_I2C_SCB_TYPE);
    Cy_SCB_I2C_Init(USER_I2C_SCB_TYPE, &g_stc_i2c_config, &g_stc_i2c_context);
    Cy_SCB_I2C_SetDataRate(USER_I2C_SCB_TYPE, E_I2C_DATARATE, E_I2C_INCLK_TARGET_FREQ);
    Cy_SCB_I2C_RegisterEventCallback(USER_I2C_SCB_TYPE, NULL, &g_stc_i2c_context);
    Cy_SCB_I2C_Enable(USER_I2C_SCB_TYPE);

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

#if NON_ISO_OPERATION == 1
static void SetISOFormat(cy_pstc_canfd_type_t canfd)
{
    /* Now a ch configured as CANFD is working. */
    canfd->M_TTCAN.unCCCR.stcField.u1INIT = 1;
    while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 1);
        /* Cancel protection by setting CCE */
    canfd->M_TTCAN.unCCCR.stcField.u1CCE = 1;
    canfd->M_TTCAN.unCCCR.stcField.u1NISO = 1;

    canfd->M_TTCAN.unCCCR.stcField.u1INIT = 0;
    while(canfd->M_TTCAN.unCCCR.stcField.u1INIT != 0);
}
#endif

/* Initialize CAN regarding pins */
static void PortInit(void)
{
    for (uint8_t i = 0; i < (sizeof(can_pin_cfg) / sizeof(can_pin_cfg[0])); i++)
    {
        Cy_GPIO_Pin_Init(can_pin_cfg[i].portReg, can_pin_cfg[i].pinNum, &can_pin_cfg[i].cfg);
    }
}

#ifdef ESTEC_CAN_ENABLE
/* CANFD reception callback */
void CAN_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg)
{
    /* Just loop back to the sender with +1 ID */
    pstcCanFDmsg->idConfig.identifier += 1u;
    Cy_CANFD_UpdateAndTransmitMsgBuffer
    (
        CY_CANFD_TYPE,
        0u,
        pstcCanFDmsg
    );
}

void CAN_RxFifoWithTopCallback(uint8_t u8FifoNum, uint8_t   u8BufferSizeInWord, uint32_t* pu32RxBuf)
{
    /*TODO*/
}

/* CANFD intrerrupt handler */
void CanfdInterruptHandler(void)
{
    /* Just invoking */
    Cy_CANFD_IrqHandler(CY_CANFD_TYPE);
}
#endif //ESTEC_CAN_ENABLE

#ifdef ESTEC_I2C_ENABLE
void Scb_I2C_IntrISR(void)
{
    /* I2C interrupt handler for High-Level APIs */
    Cy_SCB_I2C_Interrupt(USER_I2C_SCB_TYPE, &g_stc_i2c_context);
}

uint8_t Scb_I2C_Master_Write(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t txBuf[2];
    txBuf[0] = reg_addr;
    txBuf[1] = reg_data;

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

#ifdef FDA806D_AMP_ENABLE
void Fda803d_AmpInit(void)
{
    uint8_t data = 0;
	uint8_t result = 0;

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

/* [] END OF FILE */
