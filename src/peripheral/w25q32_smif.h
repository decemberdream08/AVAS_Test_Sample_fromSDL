
#ifndef _W25Q32_SMIF_H_
#define _W25Q32_SMIF_H_

#define W25_CMD_WREN          0x06
#define W25_CMD_RDSR1         0x05
#define W25_CMD_RDSR2         0x35
#define W25_CMD_WRSR2         0x31
#define W25_CMD_PP            0x02
#define W25_CMD_SE_4K         0x20
#define W25_CMD_FAST_READ_Q   0x6B
#define W25_CMD_JEDEC_ID      0x9F

#define W25_CMD_Chip_Erase    0xC7


#define W25_PAGE_SIZE         256u
#define W25_SECTOR_SIZE       4096u

#define W25_CMD_QUAD_PAGE_PROGRAM  0x32

#define W25Q32_TOTAL_SIZE (4 * 1024 * 1024UL) // 4MB

void W25_Init(void);
void W25_ReadJedecId(uint8_t id[3]);
void W25_EnableQuadIfNeeded(void);
void SMIF_UserInterruptHandler(void);

cy_en_smif_status_t W25_Read(uint32_t addr, uint8_t *buf, uint32_t len);
cy_en_smif_status_t W25_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len);
cy_en_smif_status_t W25_SectorErase4K(uint32_t addr);

cy_en_smif_status_t Flash_Write(uint32_t addr, const uint8_t *data, uint32_t len);
cy_en_smif_status_t Flash_Read(uint32_t addr, uint8_t *data, uint32_t len);

cy_en_smif_status_t Flash_EraseAll(void);
cy_en_smif_status_t Flash_EraseAllVerify(void);
cy_en_smif_status_t Flash_WriteAll(const uint8_t *data);


#endif //_W25Q32_SMIF_H_

