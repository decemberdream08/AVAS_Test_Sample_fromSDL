/***************************************************************************//**
* \file w25q32_scb.c
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
#include "main_config.h"
#include "w25q32_scb.h"

//MACRO & Define ***********************************************************************/
#ifdef ESTEC_FLASH_SPI_ENABLE
/* User setting value */
#if (CY_USE_PSVP == 1)  
  #define SOURCE_CLOCK_FRQ 24000000ul
#else
  #define SOURCE_CLOCK_FRQ 80000000ul
#endif
#define SCB_SPI_BAUDRATE     50000ul /* Please set baudrate value of SPI you want */
#define SCB_SPI_OVERSAMPLING 8ul     /* Please set oversampling of SPI you want */
#define SCB_SPI_CLOCK_FREQ   (SCB_SPI_BAUDRATE * SCB_SPI_OVERSAMPLING)

#define DIVIDER_NO_1 (1u)

#define PAGE_WRITE_MAX_SIZE		(256)

//Function & Declaration & definition ***********************************************************************/
static void SpiInterruptHandler(void);
static void SetPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum);

static uint8_t GetStatusW25Q32Flash(void);
static uint8_t GetWEL_W25Q32Flash(void);
static uint8_t GetWIP_W25Q32Flash(void);
static void WriteEnableW25Q32Flash(void);

/* SPi Context holder */
cy_stc_scb_spi_context_t spiCtx;

static const cy_stc_gpio_pin_prt_config_t g_ext_flash_pin_cfg[] =
{
//  {                 portReg,                 pinNum, outVal,         driveMode,                      hsiom, intEdge, intMask, vtrip, slewRate, driveSel },
    { EXT_FLASH_SCB_MISO_PORT, EXT_FLASH_SCB_MISO_PIN,    0ul,  CY_GPIO_DM_HIGHZ, EXT_FLASH_SCB_MISO_PIN_MUX,     0ul,     0ul,   0ul,      0ul,     0ul, },
    { EXT_FLASH_SCB_MOSI_PORT, EXT_FLASH_SCB_MOSI_PIN,    0ul, CY_GPIO_DM_STRONG, EXT_FLASH_SCB_MOSI_PIN_MUX,     0ul,     0ul,   0ul,      0ul,     0ul, },
    {  EXT_FLASH_SCB_SCK_PORT,  EXT_FLASH_SCB_SCK_PIN,    0ul, CY_GPIO_DM_STRONG,  EXT_FLASH_SCB_SCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,     0ul, },
    { EXT_FLASH_SCB_SSEL_PORT, EXT_FLASH_SCB_SSEL_PIN,    1ul, CY_GPIO_DM_STRONG, EXT_FLASH_SCB_SSEL_PIN_MUX,     0ul,     0ul,   0ul,      0ul,     0ul, },
    {       EXT_FLASH_WP_PORT,       EXT_FLASH_WP_PIN,    1ul, CY_GPIO_DM_STRONG,       EXT_FLASH_WP_PIN_MUX,     0ul,     0ul,   0ul,      0ul,     0ul, },
};

cy_stc_scb_spi_config_t spiCfg = 
    {
        .spiMode                    = CY_SCB_SPI_MASTER,
        .subMode                    = CY_SCB_SPI_MOTOROLA,
        .sclkMode                   = CY_SCB_SPI_CPHA0_CPOL0,
        .oversample                 = 8u,
        .rxDataWidth                = 8u,
        .txDataWidth                = 8u,
        .enableMsbFirst             = true,
        .enableFreeRunSclk          = false,
        .enableInputFilter          = false,
        .enableMisoLateSample       = false,
        .enableTransferSeperation   = false,
        .ssPolarity0                = CY_SCB_SPI_ACTIVE_LOW,
        .ssPolarity1                = CY_SCB_SPI_ACTIVE_LOW,
        .ssPolarity2                = CY_SCB_SPI_ACTIVE_LOW,
        .ssPolarity3                = CY_SCB_SPI_ACTIVE_LOW,
        .enableWakeFromSleep        = false,
        .txFifoTriggerLevel         = 0,
        .rxFifoTriggerLevel         = 1,
        .rxFifoIntEnableMask        = 0,
        .txFifoIntEnableMask        = 0,
        .masterSlaveIntEnableMask   = 0,
        .enableSpiDoneInterrupt     = false,
        .enableSpiBusErrorInterrupt = false,
    };
    

#define EXT_FLASH_PORT_NUM (sizeof(g_ext_flash_pin_cfg)/sizeof(g_ext_flash_pin_cfg[0]))

//Function Definition ***********************************************************************/
void Spi_SCB3_Init(void)
{
	uint16_t i;
	
    /* Initialize ports. */
    Cy_GPIO_Multi_Pin_Init(g_ext_flash_pin_cfg, EXT_FLASH_PORT_NUM);

    /* Initialize SPI */
    Cy_SysClk_PeriphAssignDivider(EXT_FLASH_SCB_PCLK, CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
    SetPeripheFracDiv24_5(SCB_SPI_CLOCK_FREQ, SOURCE_CLOCK_FRQ, DIVIDER_NO_1);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(EXT_FLASH_SCB_PCLK), CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);

    /* De-initialize SPI */
    Cy_SCB_SPI_DeInit(EXT_FLASH_SCB_TYPE);
    
    /* Setup the IRQ */
    cy_stc_sysint_irq_t irq_cfg;
    irq_cfg = (cy_stc_sysint_irq_t){
        .sysIntSrc  = EXT_FLASH_SCB_IRQN,
        .intIdx     = CPUIntIdx2_IRQn,
        .isEnabled  = true,
    };
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, SpiInterruptHandler);
    NVIC_EnableIRQ(CPUIntIdx2_IRQn);

    /* Initialize the SPI */
    Cy_SCB_SPI_Init(EXT_FLASH_SCB_TYPE, &spiCfg, &spiCtx);
    Cy_SCB_SPI_SetActiveSlaveSelect(EXT_FLASH_SCB_TYPE, EXT_FLASH_SCB_SS_IDX);
    Cy_SCB_SPI_Enable(EXT_FLASH_SCB_TYPE);

#ifdef ESTEC_GPIO_ENABLE
	W25Q32Flash_WP_Pin_Control(false); //Disable Flash Pin write protection
#endif

	W25Q32Flash_SW_Reset(); //RESET Flash

	CY_ASSERT(ReadJEDECID_W25Q32Flash() == 0xEF); //Check SPI communication - Just check W25Q32 Flash's ID(0xEF)

	/* Test 1. Chip Erase(4MB) & Noraml Read Byte */
	{
		EraseChipW25Q32Flash();

		//Read data. Just check 256 byte from address 0 ~ address 256 whether all data is clear properly to 0xff or not
		for(i = 0u; i < PAGE_WRITE_MAX_SIZE; i++)
	    {
	        static volatile uint8_t readData[PAGE_WRITE_MAX_SIZE];

			/* Read from Flash */
	        readData[i] = ReadByteW25Q32Flash(i);
	        
	        /* Verify read data */
	        CY_ASSERT(readData[i] == 0xff);
		}
	}
	
    /* Test 2. Byte Write(Write 256 byte one by one from address of 0 to 255) & Noraml Read byte */
    for(uint16_t i = 0u; i < PAGE_WRITE_MAX_SIZE; i++)
    {
        static volatile uint8_t readData1[PAGE_WRITE_MAX_SIZE];

        /* Writing 1 Byte data */
        WriteByteW25Q32Flash(i,(uint8_t)i);

        /* Read data. Just check 256 byte from address 0 ~ address 256 whether all data is written with proper value or not*/
        readData1[i] = ReadByteW25Q32Flash(i);

        /* Verify read data */
        CY_ASSERT(readData1[i] == i);

    }

	/* Test 3. Erase a Block 1(64KB) & Noraml Read 256 byte */
    {
		Erase64KBlockW25Q32Flash(FLASH_START_ADDRESS);

		//Just check 256 byte from address 0 ~ address 256 whether all data is clear properly to 0xff or not
		{
			static volatile uint8_t readData2[PAGE_WRITE_MAX_SIZE];
			int32_t check_sum = 0;

			/* Read from Flash */
	        check_sum = ReadPageDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)readData2);

			CY_ASSERT(check_sum != -1); //-1 is Error
			
			for(i = 0u; i < PAGE_WRITE_MAX_SIZE; i++)
		    {	        
		        /* Verify read data */
		        CY_ASSERT(readData2[i] == 0xff);
			}
		}
	}
	
    /* Test 4. Write a page(Max 256 byte /It's possible from 1byte to 256byte) & Fast Read 256 byte  */
    {
        static volatile uint8_t writeBuffer[PAGE_WRITE_MAX_SIZE], readData3[PAGE_WRITE_MAX_SIZE];

        /* prepare write buffer data */
        for(uint32_t j = 0ul; j < PAGE_WRITE_MAX_SIZE; j++)
        {
            writeBuffer[j] = j;
        }

		WritePageDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)writeBuffer);

        FastReadDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)readData3);

        CY_ASSERT(memcmp((const void *)writeBuffer, (const void *)readData3, PAGE_WRITE_MAX_SIZE) == 0);
    }

	/* Test 5. Erase(32KB) & Noraml Read 256 byte */
	Erase32KBlockW25Q32Flash(FLASH_START_ADDRESS);

	//Just check 256 byte from address 0 ~ address 256 whether all data is clear properly to 0xff or not
	{
		static volatile uint8_t readData4[PAGE_WRITE_MAX_SIZE];
		int32_t check_sum1 = 0;

		/* Read from Flash */
		check_sum1 = ReadPageDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)readData4);

		CY_ASSERT(check_sum1 != -1); //-1 is Error
		
		for(i = 0u; i < PAGE_WRITE_MAX_SIZE; i++)
		{			
			/* Verify read data */
			CY_ASSERT(readData4[i] == 0xff);
		}
	}

	/* This is just for Test6. Write page(Max 256 byte) */
    {
        static volatile uint8_t writeBuffer2[PAGE_WRITE_MAX_SIZE], readData5[PAGE_WRITE_MAX_SIZE];

        /* prepare write buffer data */
        for(uint32_t j = 0ul; j < PAGE_WRITE_MAX_SIZE; j++)
        {
            writeBuffer2[j] = j;
        }

		WritePageDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)writeBuffer2);

        ReadPageDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)readData5);

        CY_ASSERT(memcmp((const void *)writeBuffer2, (const void *)readData5, PAGE_WRITE_MAX_SIZE) == 0);
    }
	
	/* Test 6. Erase a sector(4KB) & Noraml Read 256 byte */
	{
		Erase4KSectorW25Q32Flash(FLASH_START_ADDRESS);
	
		//Just check 256 byte from address 0 ~ address 256 whether all data is clear properly to 0xff or not
		{
			static volatile uint8_t readData6[PAGE_WRITE_MAX_SIZE];
			int32_t check_sum2 = 0;
	
			/* Read from Flash */
			check_sum2 = ReadPageDataW25Q32Flash(FLASH_START_ADDRESS, 256, (uint8_t *)readData6);
	
			CY_ASSERT(check_sum2 != -1); //-1 is Error
			
			for(i = 0u; i < PAGE_WRITE_MAX_SIZE; i++)
			{			
				/* Verify read data */
				CY_ASSERT(readData6[i] == 0xff);
			}
		}
	}
}

/* SPI Interrupt Handler */
static void SpiInterruptHandler(void)
{
    Cy_SCB_SPI_Interrupt(EXT_FLASH_SCB_TYPE, &spiCtx);
    NVIC_ClearPendingIRQ(CPUIntIdx2_IRQn);
}

static void SetPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
    uint64_t temp = ((uint64_t)sourceFreq << 5ull);
    uint32_t divSetting;

    divSetting = (uint32_t)(temp / targetFreq);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(EXT_FLASH_SCB_PCLK), 
                                   CY_SYSCLK_DIV_24_5_BIT, divNum, 
                                   (((divSetting >> 5ul) & 0x00000FFFul) - 1ul), 
                                   (divSetting & 0x0000001Ful));
}

/**********************************************************************
 * @brief		void WriteByteW25Q32Flash(uint32_t address, uint8_t data) //address : 24bit
 * @Description	Write one byte data in W25Q32 flash (up to max 256 byte)
 *				
 * @param[in]	address = Start address(24bit) of write data
 *           	data = write data
 * @return 		None
 *				
 **********************************************************************/
void WriteByteW25Q32Flash(uint32_t address, uint8_t data) //address : 24bit
{
    uint8_t txData[5];
    uint8_t rxData[5];
	uint32_t status;
	
    txData[0] = W25Q32_FLASH_WRITE_CMD;
	txData[1] = (uint8_t)((address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(address & 0xFF);
    txData[4] = data;

	/* Enable writing */
    do
    {
        WriteEnableW25Q32Flash(); //Write Enable(WEL = 1)
    } while(GetWEL_W25Q32Flash() == 0u);
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 5u, &spiCtx);
    /* wait for completion */
    do
    {
	    status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	/* Waiting if previous writing is in progress */
    while(GetWIP_W25Q32Flash() == 1u);
}

/**********************************************************************
 * @brief		uint8_t ReadByteW25Q32Flash(uint32_t address) //address : 24bit
 * @Description	read one byte data in W25Q32 flash
 *				
 * @param[in]	address = Start address(24bit) of write data
 *           	
 * @return 		rxData[4] : one byte read data
 *				
 **********************************************************************/
uint8_t ReadByteW25Q32Flash(uint32_t address) //address : 24bit
{
    uint8_t txData[5];
    uint8_t rxData[5];
	uint32_t status;

    txData[0] = W25Q32_FLASH_READ_CMD;
    txData[1] = (uint8_t)((address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(address & 0xFF);
	txData[4] = 0u;

    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 5u, &spiCtx);
    /* wait for completion */
    do
    {
            status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);
    
    return rxData[4]; //return one byte read data
}

/**********************************************************************
 * @brief		int32_t WritePageDataW25Q32Flash(uint32_t address, uint16_t write_size, uint8_t *write_buf)
 * @Description	Write data of W25Q32 flash (up to max 256 byte)
 *				
 * @param[in]	address = Start address(24bit) of write data
 *           	write_size = Size of write data
 *				*write_buf = Save write data
 * @return 		-1 : NG
 *				 0 : OK
 **********************************************************************/
int32_t WritePageDataW25Q32Flash(uint32_t address, uint16_t write_size, uint8_t *write_buf) //address : 24bit
{
    uint8_t txData[260]; //Max 256 size + 4byte(cmd + address)
    uint8_t rxData[260]; //Max 256 size + 4byte(cmd + address)
	uint16_t i;
	uint32_t status;

    txData[0] = W25Q32_FLASH_WRITE_CMD;
    txData[1] = (uint8_t)((address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(address & 0xFF);

	CY_ASSERT(write_size <= 256); //Fail
	
	for(i=0; i<write_size; i++)
	{
		txData[i+4] = *(write_buf + i);
	}

	/* Enable writing */
    do
    {
        WriteEnableW25Q32Flash(); //Write Enable(WEL = 1)
    } while(GetWEL_W25Q32Flash() == 0u);
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, txData, rxData, write_size+4, &spiCtx);
	
    /* wait for completion */
    do
    {
            status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	/* Waiting if previous writing is in progress */
    while(GetWIP_W25Q32Flash() == 1u);
		
    return 0;
}

/**********************************************************************
 * @brief		int8_t ReadPageDataW25Q32Flash(uint32_t address, uint16_t read_size, uint8_t *read_buf)
 * @Description	Read data of W25Q32 flash (up to max 256 byte)
 *				
 * @param[in]	address = Start address(24bit) of read data
 *           	read_size = Size of read data
 *				*read_buf = Save read data
 * @return 		check_sum = check sum of all read data (-1 : Fail)
 *				
 **********************************************************************/
int32_t ReadPageDataW25Q32Flash(uint32_t address, uint16_t read_size, uint8_t *read_buf) //address : 24bit
{
    uint8_t txData[260]; //Max 256 size + 4byte(cmd + address)
    uint8_t rxData[260]; //Max 256 size + 4byte(cmd + address)
	uint16_t i;
	uint32_t status, check_sum = 0;

    txData[0] = W25Q32_FLASH_READ_CMD;
    txData[1] = (uint8_t)((address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(address & 0xFF);

	CY_ASSERT(read_size >= 1); //Fail
		
	for(i=4; i<read_size+4; i++)
	{
		txData[i] = 0u;
	}
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, txData, rxData, read_size+4, &spiCtx);
	
    /* wait for completion */
    do
    {
    	status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	memcpy(read_buf, &rxData[4], read_size);

	for(i=0;i<read_size;i++)
		check_sum += *(read_buf + i);
	
    return check_sum;
}


/**********************************************************************
 * @brief		int32_t FastReadDataW25Q32Flash(uint32_t address, uint16_t read_size, uint8_t *read_buf) //address : 24bit
 * @Description	Read data of W25Q32 flash (up to max 256 byte)
 *				
 * @param[in]	address = Start address(24bit) of read data
 *           	read_size = Size of read data
 *				*read_buf = Save read data
 * @return 		check_sum = check sum of all read data (-1 : Fail)
 *				
 **********************************************************************/
int32_t FastReadDataW25Q32Flash(uint32_t address, uint16_t read_size, uint8_t *read_buf) //address : 24bit
{
    uint8_t txData[261]; //Max 256 size + 4byte(cmd + address) + 1byte(dummy)
    uint8_t rxData[261]; //Max 256 size + 4byte(cmd + address) + 1byte(dummy)
	uint16_t i;
	uint32_t status, check_sum = 0;

    txData[0] = W25Q32_FLASH_FAST_READ_CMD;
    txData[1] = (uint8_t)((address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(address & 0xFF);
	txData[4] = 0;

	CY_ASSERT(read_size >= 1); //Fail
		
	for(i=5; i<read_size+5; i++)
	{
		txData[i] = 0u;
	}
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, txData, rxData, read_size+5, &spiCtx);
	
    /* wait for completion */
    do
    {
    	status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	memcpy(read_buf, &rxData[5], read_size);

	for(i=0;i<read_size;i++)
		check_sum += *(read_buf + i);
	
    return check_sum;
}

/**********************************************************************
 * @brief		uint8_t ReadJEDECID_W25Q32Flash(void)
 * @Description	Read JEDEC_ID of W25Q32 flash 
 *				
 * @param[in]	None
 *              
 * @return 		rxData[1] : W25Q32_FLASH_MANUFACTURE_ID_CMD(0xEF)
 *				
 **********************************************************************/
uint8_t ReadJEDECID_W25Q32Flash(void)
{
    /* RDSR */
    uint8_t txData[4];
    uint8_t rxData[4];
	uint32_t status;
	
    txData[0] = W25Q32_FLASH_JEDEC_ID_CMD;
    txData[1] = 0u;
	txData[2] = 0u;
	txData[3] = 0u;
    
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 4u, &spiCtx);
	
    /* wait for completion */
    do
    {
            status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);
    
    return rxData[1]; //return W25Q32_FLASH_MANUFACTURE_ID_CMD
}

static uint8_t GetStatusW25Q32Flash(void)
{
    /* RDSR */
    uint8_t txData[3];
    uint8_t rxData[3];

    txData[0] = W25Q32_FLASH_RDSR1_CMD;
    txData[1] = 0u;
	txData[2] = 0u;

    uint32_t status;
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 3u, &spiCtx);
    /* wait for completion */
    do
    {
            status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);
    
    return rxData[1];
}


static uint8_t GetWEL_W25Q32Flash(void)
{
    return((GetStatusW25Q32Flash() & 0x02u) >> 1u);
}

static uint8_t GetWIP_W25Q32Flash(void) //WRITE IN PROGRESS : busy
{
    return(GetStatusW25Q32Flash() & 0x01u);
}

static void WriteEnableW25Q32Flash(void)
{
    uint8_t txData = W25Q32_FLASH_WREN_CMD;
    uint8_t rxData;
    uint32_t status;

    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 1u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);
}

void W25Q32Flash_SW_Reset(void)
{
    uint8_t txData;
    uint8_t rxData;
    uint32_t status;

	txData = W25Q32_FLASH_ENABLE_RESET_CMD;
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 1u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	txData = W25Q32_FLASH_RESET_DEVICE_CMD;
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 1u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);
}

/**********************************************************************
 * @brief		void Erase4KSectorW25Q32Flash(uint32_t erase_address)
 * @Description	Erase specific Sector(4KB) of erase address in W25Q32 flash
 *				
 * @param[in]	erase_address = 24bit address(shoud be 0x1000 x 0~1,024)
 *              
 * @return 		None
 *				
 **********************************************************************/
void Erase4KSectorW25Q32Flash(uint32_t erase_address)
{
    uint8_t txData[4];
    uint8_t rxData[4];
    uint32_t status;

	CY_ASSERT(((erase_address % 0x1000) || ((erase_address / 0x1000) > 1024)) == false); //erase_address is invalid.
	
	txData[0] = W25Q32_FLASH_4KB_SECTOR_ERASE_CMD;
	txData[1] = (uint8_t)((erase_address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((erase_address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(erase_address & 0xFF);

	/* Enable writing */
    do
    {
        WriteEnableW25Q32Flash(); //Write Enable(WEL = 1)
    } while(GetWEL_W25Q32Flash() == 0u);
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 4u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	/* Waiting if previous writing is in progress */
    while(GetWIP_W25Q32Flash() == 1u);
}

/**********************************************************************
 * @brief		void Erase32KBlockW25Q32Flash(uint32_t erase_address)
 * @Description	Erase specific sector(32KB) of erase address in W25Q32 flash
 *				
 * @param[in]	erase_address = 24bit address(shoud be 0x10000/2 x 0~126)
 *              
 * @return 		None
 *				
 **********************************************************************/
void Erase32KBlockW25Q32Flash(uint32_t erase_address)
{
    uint8_t txData[4];
    uint8_t rxData[4];
    uint32_t status;

	CY_ASSERT(((erase_address % (0x10000/2)) || ((erase_address / (0x10000/2) > 126))) == false); //erase_address is invalid.
		
	txData[0] = W25Q32_FLASH_32KB_BLOCK_ERASE_CMD;
	txData[1] = (uint8_t)((erase_address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((erase_address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(erase_address & 0xFF);

	/* Enable writing */
    do
    {
        WriteEnableW25Q32Flash(); //Write Enable(WEL = 1)
    } while(GetWEL_W25Q32Flash() == 0u);
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 4u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	/* Waiting if previous writing is in progress */
    while(GetWIP_W25Q32Flash() == 1u);
}


/**********************************************************************
 * @brief		void Erase64KBlockW25Q32Flash(uint32_t erase_address)
 * @Description	Erase specific sector(64KB) of erase address in W25Q32 flash
 *				
 * @param[in]	erase_address = 24bit address(shoud be 0x10000 x 0~63)
 *              
 * @return 		None
 *				
 **********************************************************************/
void Erase64KBlockW25Q32Flash(uint32_t erase_address)
{
    uint8_t txData[4];
    uint8_t rxData[4];
    uint32_t status;

	CY_ASSERT(((erase_address % 0x10000) || ((erase_address / 0x10000) > 63)) == false); 
	
	txData[0] = W25Q32_FLASH_64KB_BLOCK_ERASE_CMD;
	txData[1] = (uint8_t)((erase_address & 0xFF0000) >> 16);
	txData[2] = (uint8_t)((erase_address & 0xFF00) >> 8);
	txData[3] = (uint8_t)(erase_address & 0xFF);

	/* Enable writing */
    do
    {
        WriteEnableW25Q32Flash(); //Write Enable(WEL = 1)
    } while(GetWEL_W25Q32Flash() == 0u);
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 4u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	/* Waiting if previous writing is in progress */
    while(GetWIP_W25Q32Flash() == 1u);
}

/**********************************************************************
 * @brief		void EraseChipW25Q32Flash(void)
 * @Description	Erase all memories (Chip erease - 4MB) in W25Q32 flash
 *				
 * @param[in]	None
 *              
 * @return 		None
 *				
 **********************************************************************/
void EraseChipW25Q32Flash(void)
{
    uint8_t txData[1];
    uint8_t rxData[1];
    uint32_t status;

	txData[0] = W25Q32_FLASH_CHIP_ERASE_CMD;
	
	/* Enable writing */
    do
    {
        WriteEnableW25Q32Flash(); //Write Enable(WEL = 1)
    } while(GetWEL_W25Q32Flash() == 0u);
	
    Cy_SCB_SPI_Transfer(EXT_FLASH_SCB_TYPE, &txData, &rxData, 1u, &spiCtx);

    /* wait for completion */
    do
    {
        status = Cy_SCB_SPI_GetTransferStatus(EXT_FLASH_SCB_TYPE, &spiCtx);
    } while((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0u);

	/* Waiting if previous writing is in progress */
    while(GetWIP_W25Q32Flash() == 1u);
}

#ifdef ESTEC_GPIO_ENABLE
/**********************************************************************
 * @brief		void W25Q32Flash_WP_Pin_Control(bool Write_Protection)
 * @Description	Control write protection pin of W25Q32 flash
 *				
 * @param[in]	Write_Protection = true : Enable write protection(GPIO Low)
 *              				false : Disable write protection(GPIO High)
 * @return 		None
 *				
 **********************************************************************/
void W25Q32Flash_WP_Pin_Control(bool Write_Protection)
{
	if(Write_Protection)
		Cy_GPIO_Clr(EXT_FLASH_WP_PORT, EXT_FLASH_WP_PIN); //Enable Write Protection
	else
		Cy_GPIO_Set(EXT_FLASH_WP_PORT, EXT_FLASH_WP_PIN); //Disable Write Protection
}
#endif

#endif //ESTEC_FLASH_SPI_ENABLE

/* [] END OF FILE */
