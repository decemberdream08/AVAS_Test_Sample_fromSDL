/***************************************************************************//**
* \file w25q32_scb.h
*
* \version 1.0
*
* \brief Main example file for CM7_0
*
********************************************************************************
* \copyright
* ESTec KMS. This code is based on Cypress reference SW(SDL).
*******************************************************************************/
#ifndef _W25Q32_SCB_H_
#define _W25Q32_SCB_H_

//MACRO & Define ***********************************************************************/
/* FLASH SPI Mux  */
#define EXT_FLASH_SCB_TYPE              SCB3

#define EXT_FLASH_SCB_MISO_PORT         GPIO_PRT13
#define EXT_FLASH_SCB_MISO_PIN          0
#define EXT_FLASH_SCB_MISO_PIN_MUX      P13_0_SCB3_SPI_MISO

#define EXT_FLASH_SCB_MOSI_PORT         GPIO_PRT13
#define EXT_FLASH_SCB_MOSI_PIN          1
#define EXT_FLASH_SCB_MOSI_PIN_MUX      P13_1_SCB3_SPI_MOSI

#define EXT_FLASH_SCB_SCK_PORT          GPIO_PRT13
#define EXT_FLASH_SCB_SCK_PIN           2
#define EXT_FLASH_SCB_SCK_PIN_MUX       P13_2_SCB3_SPI_CLK

#define EXT_FLASH_SCB_SSEL_PORT         GPIO_PRT13
#define EXT_FLASH_SCB_SSEL_PIN          3
#define EXT_FLASH_SCB_SSEL_PIN_MUX      P13_3_SCB3_SPI_SELECT0

#define EXT_FLASH_SCB_SS_IDX            CY_SCB_SPI_SLAVE_SELECT0

#define EXT_FLASH_WP_PORT               GPIO_PRT13
#define EXT_FLASH_WP_PIN                5
#define EXT_FLASH_WP_PIN_MUX            P13_5_GPIO

#define EXT_FLASH_SCB_PCLK              PCLK_SCB3_CLOCK
#define EXT_FLASH_SCB_IRQN              scb_3_interrupt_IRQn

/* External W25Q32 Flash Commnad */

#define W25Q32_FLASH_READ_CMD   		0x03 // Read Data CMD(03h)
#define W25Q32_FLASH_WRITE_CMD  		0x02 // Page Program CMD(02h) //Page program 1 byte ~  256byte
#define W25Q32_FLASH_WRDI_CMD   		0x04 // Write Disable CMD(04h)
#define W25Q32_FLASH_WREN_CMD   		0x06 // Write Enable CMD(06h)

#define W25Q32_FLASH_RDSR1_CMD   		0x05 // Read Status Register1 CMD(05h)
#define W25Q32_FLASH_RDSR2_CMD   		0x35 // Read Status Register2 CMD(35h)
#define W25Q32_FLASH_RDSR3_CMD   		0x15 // Read Status Register3 CMD(15h)
#define W25Q32_FLASH_WRSR1_CMD   		0x01 // Wirte Status Register1 CMD(01h)
#define W25Q32_FLASH_WRSR2_CMD   		0x31 // Wirte Status Register2 CMD(31h)
#define W25Q32_FLASH_WRSR3_CMD   		0x11 // Wirte Status Register3 CMD(11h)

#define W25Q32_FLASH_FAST_READ_CMD		0x0B // Fast Read CMD(0Bh)

#define W25Q32_FLASH_ENABLE_RESET_CMD	0x66 // Enable Reset CMD(66h)
#define W25Q32_FLASH_RESET_DEVICE_CMD	0x99 // Reset Device CMD(99h)

#define W25Q32_FLASH_JEDEC_ID_CMD 		0x9F // Read JEDEC ID CMD(9Fh)

#define W25Q32_FLASH_4KB_SECTOR_ERASE_CMD			0x20 // 4KB Sector Erase CMD(20h)
#define W25Q32_FLASH_32KB_BLOCK_ERASE_CMD		0x52 // 32KB Block Erase CMD(52h)
#define W25Q32_FLASH_64KB_BLOCK_ERASE_CMD				0xD8 // 64KB Block Erase CMD(D8h)
#define W25Q32_FLASH_CHIP_ERASE_CMD				0xC7 // Chip Erase(0xC7 or 0x60)

#define W25Q32_FLASH_MANUFACTURE_ID_CMD			0xEF // Manufacture ID

/* External W25Q32 Flash Size */
#define FLASH_BLOCK_COUNT 		0x40 // W25Q32 block counts = 64. (=4Mbyte / 64Kbyte)
#define FLASH_BLOCK_SIZE	 	0x10000 // W25Q32 block size = 64Kbyte
#define FLASH_PAGE_SIZE			0x100 //W25Q32 Page size = 256byte

/* Flash Start Address */
#define FLASH_START_ADDRESS			0x000000
#define FLASH_END_ADDRESS			0x3FFFFF
#define FLASH_BLOCK_BLOCK_SIZE		0x10000  
#define FLAHS_BLOCK_LAST_COUNT		63

#define FLASH_SECTOR_0_START		0x00000
#define FLASH_SECTOR_1_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_2_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_3_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_4_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_5_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_6_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_7_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_8_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_9_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_10_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_11_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_12_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_13_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_14_START		(FLASH_SECTOR_0_START + 0x1000)
#define FLASH_SECTOR_15_START		(FLASH_SECTOR_0_START + 0x1000)


//Function Declaration ***********************************************************************/
void Spi_SCB3_Init(void); /* Init function */
void W25Q32Flash_SW_Reset(void);

#ifdef ESTEC_GPIO_ENABLE
void W25Q32Flash_WP_Pin_Control(bool Write_Protection);
#endif

int32_t WritePageDataW25Q32Flash(uint32_t address, uint16_t write_size, uint8_t *write_buf); //address : 24bit
int32_t ReadPageDataW25Q32Flash(uint32_t address, uint16_t read_size, uint8_t *read_buf); //address : 24bit
void WriteByteW25Q32Flash(uint32_t address, uint8_t data);
uint8_t ReadByteW25Q32Flash(uint32_t address);

int32_t FastReadDataW25Q32Flash(uint32_t address, uint16_t read_size, uint8_t *read_buf); //address : 24bit
uint8_t ReadJEDECID_W25Q32Flash(void);

void Erase4KSectorW25Q32Flash(uint32_t erase_address);
void Erase32KBlockW25Q32Flash(uint32_t erase_address);
void Erase64KBlockW25Q32Flash(uint32_t erase_address);
void EraseChipW25Q32Flash(void);

#endif //_W25Q32_SCB_H_

