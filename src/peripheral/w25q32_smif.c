#include "cy_project.h"
#include "cy_device_headers.h"
#include "main_config.h"

#include "w25q32_smif.h"

#ifdef ESTEC_FALSH_QSPI_ENABLE

#define CY_SMIF0_TYPE                           SMIF0
#define CY_SMIF0_DEVICE0                        SMIF0_DEVICE0
#define CY_SMIF0_DEVICE1                        SMIF0_DEVICE1
  
#define CY_SMIF0_CLK_PORT                       GPIO_PRT6
#define CY_SMIF0_CLK_PIN                        3
#define CY_SMIF0_CLK_PIN_MUX                    P6_3_SMIF0_SPIHB_CLK
  
#define CY_SMIF0_CLK_INV_PORT                   NULL
#define CY_SMIF0_CLK_INV_PIN                    0
#define CY_SMIF0_CLK_INV_PIN_MUX                HSIOM_SEL_GPIO
  
#define CY_SMIF0_RWDS_PORT                      GPIO_PRT6
#define CY_SMIF0_RWDS_PIN                       4
#define CY_SMIF0_RWDS_PIN_MUX                   P6_4_SMIF0_SPIHB_RWDS
  
#define CY_SMIF0_SELECT0_PORT                   GPIO_PRT6
#define CY_SMIF0_SELECT0_PIN                    5
#define CY_SMIF0_SELECT0_PIN_MUX                P6_5_SMIF0_SPIHB_SELECT0
  
#define CY_SMIF0_SELECT1_PORT                   GPIO_PRT7
#define CY_SMIF0_SELECT1_PIN                    0
#define CY_SMIF0_SELECT1_PIN_MUX                P7_0_SMIF0_SPIHB_SELECT1
  
#define CY_SMIF0_DATA0_PORT                     GPIO_PRT7
#define CY_SMIF0_DATA0_PIN                      1
#define CY_SMIF0_DATA0_PIN_MUX                  P7_1_SMIF0_SPIHB_DATA0
  
#define CY_SMIF0_DATA1_PORT                     GPIO_PRT7
#define CY_SMIF0_DATA1_PIN                      2
#define CY_SMIF0_DATA1_PIN_MUX                  P7_2_SMIF0_SPIHB_DATA1
  
#define CY_SMIF0_DATA2_PORT                     GPIO_PRT7
#define CY_SMIF0_DATA2_PIN                      3
#define CY_SMIF0_DATA2_PIN_MUX                  P7_3_SMIF0_SPIHB_DATA2
  
#define CY_SMIF0_DATA3_PORT                     GPIO_PRT7
#define CY_SMIF0_DATA3_PIN                      4
#define CY_SMIF0_DATA3_PIN_MUX                  P7_4_SMIF0_SPIHB_DATA3


#define CY_SMIF_DRV_SMIF0_CORE0                         SMIF0

#define SMIF_USED               CY_SMIF_DRV_SMIF0_CORE0

#define SMIF_DEVICE0            CY_SMIF_DRV_SMIF0_CORE0_DEVICE0
#define SMIF_DEVICE1            CY_SMIF_DRV_SMIF0_CORE0_DEVICE1

#define SMIF_IRQ_NO             CY_SMIF_DRV_SMIF0_IRQN

#define CY_SMIF_CLK_PORT        CY_SMIF0_CLK_PORT
#define CY_SMIF_CLK_PIN         CY_SMIF0_CLK_PIN
#define CY_SMIF_CLK_PIN_MUX     CY_SMIF0_CLK_PIN_MUX

#ifdef CY_SMIF0_CLK_INV_PORT
#define CY_SMIF_CLK_INV_PORT    CY_SMIF0_CLK_INV_PORT
#define CY_SMIF_CLK_INV_PIN     CY_SMIF0_CLK_INV_PIN
#define CY_SMIF_CLK_INV_PIN_MUX CY_SMIF0_CLK_INV_PIN_MUX
#else
#define CY_SMIF_CLK_INV_PORT    (NULL)         // dummy
#define CY_SMIF_CLK_INV_PIN     0              // dummy
#define CY_SMIF_CLK_INV_PIN_MUX HSIOM_SEL_GPIO // dummy
#endif

#define CY_SMIF_SELECT0_PORT    CY_SMIF0_SELECT0_PORT
#define CY_SMIF_SELECT0_PIN     CY_SMIF0_SELECT0_PIN
#define CY_SMIF_SELECT0_PIN_MUX CY_SMIF0_SELECT0_PIN_MUX

#define CY_SMIF_SELECT1_PORT    CY_SMIF0_SELECT1_PORT
#define CY_SMIF_SELECT1_PIN     CY_SMIF0_SELECT1_PIN
#define CY_SMIF_SELECT1_PIN_MUX CY_SMIF0_SELECT1_PIN_MUX

#define CY_SMIF_DATA0_PORT      CY_SMIF0_DATA0_PORT
#define CY_SMIF_DATA0_PIN       CY_SMIF0_DATA0_PIN
#define CY_SMIF_DATA0_PIN_MUX   CY_SMIF0_DATA0_PIN_MUX

#define CY_SMIF_DATA1_PORT      CY_SMIF0_DATA1_PORT
#define CY_SMIF_DATA1_PIN       CY_SMIF0_DATA1_PIN
#define CY_SMIF_DATA1_PIN_MUX   CY_SMIF0_DATA1_PIN_MUX

#define CY_SMIF_DATA2_PORT      CY_SMIF0_DATA2_PORT
#define CY_SMIF_DATA2_PIN       CY_SMIF0_DATA2_PIN
#define CY_SMIF_DATA2_PIN_MUX   CY_SMIF0_DATA2_PIN_MUX

#define CY_SMIF_DATA3_PORT      CY_SMIF0_DATA3_PORT
#define CY_SMIF_DATA3_PIN       CY_SMIF0_DATA3_PIN
#define CY_SMIF_DATA3_PIN_MUX   CY_SMIF0_DATA3_PIN_MUX

#define CY_SMIF_CMD_NOT_LAST        ((cy_en_smif_cmd_last_t)0u)
#define CY_SMIF_CMD_WITH_LAST_BYTE  ((cy_en_smif_cmd_last_t)1u)

typedef enum
{
    MPU_REG_BG = 0,
    MPU_REG_SRAM,
    MPU_REG_FLASH,
    MPU_REG_SMIF0_DEV,
    MPU_REG_SMIF1_DEV,
    MPU_REG_REGISTER,
} en_mpu_region_name_t;

#define W25_SMIF_BASE         SMIF_USED
#define W25_SLAVE             (CY_SMIF_SLAVE_SELECT_0)

#define SMIF_MPU_REG_NO  MPU_REG_SMIF0_DEV
#define SMIF_HF_CLOCK  CY_SYSCLK_HFCLK_6

#define PLL_200M_0_PATH_NO				(3u)
static void ChangePLLFrequency(uint32_t outputFreq)
{
  #if (CY_USE_PSVP == 0u)
    cy_stc_pll_config_t pll200_0_Config = 
    {
      .inputFreq	   = 16000000ul,
      .outputFreq    = outputFreq,	// target PLL output
      .lfMode 	   = 0u,			// VCO frequency is [200MHz, 400MHz]
      .outputMode    = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
    };

    CY_ASSERT(Cy_SysClk_HfClockSetSource(CY_SYSCLK_HFCLK_0, CY_SYSCLK_HFCLK_IN_CLKPATH5) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllDisable(PLL_200M_0_PATH_NO) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_ClkPathSetSource(PLL_200M_0_PATH_NO, CY_SYSCLK_CLKPATH_IN_ECO) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllConfigure(PLL_200M_0_PATH_NO , &pll200_0_Config) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllEnable(PLL_200M_0_PATH_NO, 10000ul) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_HfClockSetSource(CY_SYSCLK_HFCLK_0, CY_SYSCLK_HFCLK_IN_CLKPATH3) == CY_SYSCLK_SUCCESS);
  #endif
	return;
}

typedef struct
{
    volatile stc_GPIO_PRT_t*  port;
    uint8_t                   pin;
    en_hsiom_sel_t            hsiom;
    uint32_t                  driveMode;
} cy_stc_smif_port_t;

cy_stc_smif_port_t smifPortCfg[] =
{
    {CY_SMIF_CLK_PORT,         CY_SMIF_CLK_PIN,       CY_SMIF_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
    {CY_SMIF_CLK_INV_PORT,     CY_SMIF_CLK_INV_PIN,   HSIOM_SEL_GPIO,          CY_GPIO_DM_STRONG_IN_OFF},
    {CY_SMIF_SELECT0_PORT,     CY_SMIF_SELECT0_PIN,   CY_SMIF_SELECT0_PIN_MUX, CY_GPIO_DM_PULLUP},
    {CY_SMIF_DATA0_PORT,       CY_SMIF_DATA0_PIN,     CY_SMIF_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA1_PORT,       CY_SMIF_DATA1_PIN,     CY_SMIF_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA2_PORT,       CY_SMIF_DATA2_PIN,     CY_SMIF_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA3_PORT,       CY_SMIF_DATA3_PIN,     CY_SMIF_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
};
#define SIZE_OF_PORT (sizeof(smifPortCfg)/sizeof(cy_stc_smif_port_t))

cy_stc_smif_context_t smifContext;

cy_stc_smif_config_t smifConfig =
{
    .txClk         = CY_SMIF_DIV_INV_FOR_SDR,
    .rxClk         = CY_SMIF_INV_OUTPUT_CLK,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_3_CLK_PULUS_MIN,
    .mode          = CY_SMIF_NORMAL,
    .blockEvent    = CY_SMIF_BUS_ERROR,
};

cy_stc_sysint_irq_t smif_irq_cfg =
{
    .sysIntSrc  = SMIF_IRQ_NO,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

void SMIF_UserInterruptHandler(void)
{
    Cy_SMIF_Interrupt(SMIF_USED, &smifContext);
}

static void SmifPortInit(cy_stc_smif_port_t cfg[], uint8_t size)
{
    cy_stc_gpio_pin_config_t pinCfg = {0};
    for(uint32_t i = 0; i < size; i++)
    {
        pinCfg.driveMode = cfg[i].driveMode;
        pinCfg.hsiom     = cfg[i].hsiom;
        Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
    }
}

static inline void wait_tx(void)
{
    while (Cy_SMIF_GetTxfrStatus(W25_SMIF_BASE, &smifContext) == CY_SMIF_SEND_BUSY) {}
}
static inline void wait_rx(void)
{
    while (Cy_SMIF_GetTxfrStatus(W25_SMIF_BASE, &smifContext) == CY_SMIF_REC_BUSY) {}
}


static void write_enable(void)
{
    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
    W25_CMD_WREN,
    false, // 1-byte command
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR, // cmd width/rate
    NULL, 0, // no params
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    W25_SLAVE,
    CY_SMIF_CMD_WITH_LAST_BYTE, // command completes here
    &smifContext);
    wait_tx();
}


static void wait_while_busy(void)
{
    uint8_t sr1 = 1u;
    while (sr1 & 1u)
    {
        (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
        W25_CMD_RDSR1,
        false,
        CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
        NULL, 0,
        CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
        W25_SLAVE,
        CY_SMIF_CMD_NOT_LAST, //?´ì–´???½ê¸°
        &smifContext);
        wait_tx();


        // read 1 byte status
        Cy_SMIF_ReceiveData(W25_SMIF_BASE,
                            &sr1,
                            1,
                            CY_SMIF_WIDTH_SINGLE,
                            CY_SMIF_SDR,
                            W25_SLAVE,
                            NULL,
                            &smifContext);

        wait_rx();
    }
}

void W25_Init(void)
{
    //flash write, read ê²€ì¦ìš© ì½”ë“œ [
    uint8_t id[3] = {0};
    volatile uint8_t ok = 1;

    uint8_t txData[W25_PAGE_SIZE];
    uint8_t rxData[W25_PAGE_SIZE];
	// ]

    /* Clock ?¤ì • */
    Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
    Cy_SysClk_HfClockSetDivider(SMIF_HF_CLOCK, CY_SYSCLK_HFCLK_DIVIDE_BY_4);

    // Please modify according to your HW condition.
    ChangePLLFrequency(100000000); // SMIF out clock will be 50,000,000

    SmifPortInit(smifPortCfg, SIZE_OF_PORT);

    /* Interrupt settings */
    Cy_SysInt_InitIRQ(&smif_irq_cfg);
    Cy_SysInt_SetSystemIrqVector(smif_irq_cfg.sysIntSrc, SMIF_UserInterruptHandler);
    NVIC_SetPriority(smif_irq_cfg.intIdx, 0);
    NVIC_EnableIRQ(smif_irq_cfg.intIdx);

    /* SMIF Init */
    Cy_SMIF_DeInit(SMIF_USED);
    Cy_SMIF_Init(SMIF_USED, &smifConfig, 1000, &smifContext);
    Cy_SMIF_DeviceSetDataSelect(SMIF_DEVICE0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_Enable(SMIF_USED, &smifContext);

    write_enable();

    //flash ?™ìž‘ ê²€ì¦ìš© ì½”ë“œ [
    W25_ReadJedecId(id);

    for (int i = 0; i < sizeof(txData); i++) txData[i] = i & 0xFF;

    // Flash Write & Read
    Flash_Write(0x00000, txData, sizeof(txData));
    Flash_Read(0x00000, rxData, sizeof(rxData));

    for (int i = 0; i < sizeof(txData); i++) {
        if (txData[i] != rxData[i]) { 
            ok = 0;
            break;
        }
    }

    Flash_EraseAll();
    Flash_EraseAllVerify(); // ëª¨ë“  ê°’ì´ 0xFF?´ë©´ ?•ìƒ
    // ]
}

void W25_ReadJedecId(uint8_t id[3])
{
    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
    W25_CMD_JEDEC_ID,
    false,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    NULL, 0,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    W25_SLAVE,
    CY_SMIF_CMD_NOT_LAST,
    &smifContext);
    wait_tx();

    Cy_SMIF_ReceiveData(W25_SMIF_BASE,
                        id,
                        3,
                        CY_SMIF_WIDTH_SINGLE,
                        CY_SMIF_SDR,
                        W25_SLAVE,
                        NULL,
                        &smifContext);

    wait_rx();
}


void W25_EnableQuadIfNeeded(void)
{
    uint8_t sr2 = 0u;

    // Read SR2
    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
    W25_CMD_RDSR2,
    false,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    NULL, 0,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    W25_SLAVE,
    CY_SMIF_CMD_NOT_LAST,
    &smifContext);
    wait_tx();
    Cy_SMIF_ReceiveData(W25_SMIF_BASE,
                        &sr2,
                        1,
                        CY_SMIF_WIDTH_SINGLE,
                        CY_SMIF_SDR,
                        W25_SLAVE,
                        NULL,
                        &smifContext);

    wait_rx();

    if ((sr2 & 0x02u) == 0u)
    {
        sr2 |= 0x02u; // QE=1
        write_enable();
        (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
        W25_CMD_WRSR2,
        false,
        CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
        &sr2, 1,
        CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
        W25_SLAVE,
        CY_SMIF_CMD_WITH_LAST_BYTE,
        &smifContext);
        wait_tx();
        wait_while_busy();
    }
}


cy_en_smif_status_t W25_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t param[4] = { (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr, 0x00/*dummy*/ };

    // 0x6B Quad-Output Fast Read: cmd (1S) + addr(1S) + dummy(1S) ??data(4S)
    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
    W25_CMD_FAST_READ_Q,
    false,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    param, 4,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    W25_SLAVE,
    CY_SMIF_CMD_NOT_LAST,
    &smifContext);
    wait_tx();

    Cy_SMIF_ReceiveData(W25_SMIF_BASE,
                        buf,
                        len,
                        CY_SMIF_WIDTH_QUAD,
                        CY_SMIF_SDR,
                        W25_SLAVE,
                        NULL,
                        &smifContext);

    wait_rx();
    return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t W25_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len)
{
    write_enable();
    uint8_t param[3] = { (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr };

    // 0x02 Page Program: cmd(1S) + addr(1S) ??data(4S) ?ˆìš© (W25Q ?œë¦¬ì¦?Quad Input Program=0x32???ˆìŒ)
    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
    /*W25_CMD_PP*/W25_CMD_QUAD_PAGE_PROGRAM,
    false,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    param, 3,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    W25_SLAVE,
    CY_SMIF_CMD_NOT_LAST,
    &smifContext);
    wait_tx();

    Cy_SMIF_TransmitData(W25_SMIF_BASE,
                         data,
                         len,
                         CY_SMIF_WIDTH_QUAD,
                         CY_SMIF_SDR,
                         W25_SLAVE,
                         NULL,
                         &smifContext);

    wait_tx();
    wait_while_busy();
    return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t W25_SectorErase4K(uint32_t addr)
{
    write_enable();
    uint8_t param[3] = { (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr };


    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
    W25_CMD_SE_4K,
    false,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    param, 3,
    CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
    W25_SLAVE,
    CY_SMIF_CMD_WITH_LAST_BYTE,
    &smifContext);
    wait_tx();
    wait_while_busy();
    return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t Flash_Write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    uint32_t currentAddr = addr;
    uint32_t remaining = len;
    const uint8_t *pSrc = data;

    // 1) ?¹í„° ?¨ìœ„ë¡??„ìš” ???? œ (4KB ?¨ìœ„)
    uint32_t startSector = addr / W25_SECTOR_SIZE;
    uint32_t endSector   = (addr + len - 1) / W25_SECTOR_SIZE;
    for (uint32_t sector = startSector; sector <= endSector; sector++)
    {
        uint32_t sectorAddr = sector * W25_SECTOR_SIZE;
        status = W25_SectorErase4K(sectorAddr);
        if (status != CY_SMIF_SUCCESS) return status;
    }

    // 2) ?˜ì´ì§€ ?¨ìœ„(256B)ë¡?ë¶„í• ?˜ì—¬ ?°ê¸°
    while (remaining > 0)
    {
        uint32_t pageOffset = currentAddr % W25_PAGE_SIZE;
        uint32_t spaceInPage = W25_PAGE_SIZE - pageOffset;
        uint32_t writeSize = (remaining < spaceInPage) ? remaining : spaceInPage;

        status = W25_PageProgram(currentAddr, pSrc, writeSize);
        if (status != CY_SMIF_SUCCESS) return status;

        currentAddr += writeSize;
        pSrc        += writeSize;
        remaining   -= writeSize;
    }

    return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t Flash_Read(uint32_t addr, uint8_t *data, uint32_t len)
{
    return W25_Read(addr, data, len);
}

cy_en_smif_status_t Flash_EraseAll(void)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    write_enable();

    (void)Cy_SMIF_TransmitCommand(W25_SMIF_BASE,
                                  W25_CMD_Chip_Erase, // Chip Erase
                                  false,
                                  CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
                                  NULL, 0,
                                  CY_SMIF_WIDTH_SINGLE, CY_SMIF_SDR,
                                  W25_SLAVE,
                                  CY_SMIF_CMD_WITH_LAST_BYTE,
                                  &smifContext);

    wait_tx();
    wait_while_busy();
    return status;
}

cy_en_smif_status_t Flash_EraseAllVerify(void)
{
    uint8_t buffer[256];
    uint32_t addr = 0;

    while (addr < W25Q32_TOTAL_SIZE)
    {
        // 256B ?¨ìœ„ë¡??½ê¸°
        if (W25_Read(addr, buffer, sizeof(buffer)) != CY_SMIF_SUCCESS)
            return CY_SMIF_GENERAL_ERROR;

        for (uint32_t i = 0; i < sizeof(buffer); i++)
        {
            if (buffer[i] != 0xFF)
            {
                // Erase ?¤íŒ¨ êµ¬ê°„ ë°œê²¬
                 return CY_SMIF_BAD_PARAM;
            }
        }

        addr += sizeof(buffer);
    }

    return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t Flash_WriteAll(const uint8_t *data)
{
    cy_en_smif_status_t status;

    // ?„ì²´ Erase ?˜í–‰
    status = Flash_EraseAll();
    if (status != CY_SMIF_SUCCESS)
        return status;

    // ?„ì²´ ?¬ê¸° 4MB
    uint32_t addr = 0;
    uint32_t remaining = W25Q32_TOTAL_SIZE;
    const uint8_t *pData = data;

    while (remaining > 0)
    {
        uint32_t writeSize = (remaining >= W25_PAGE_SIZE) ? W25_PAGE_SIZE : remaining;
        status = W25_PageProgram(addr, pData, writeSize);
        if (status != CY_SMIF_SUCCESS)
            return status;

        addr += writeSize;
        pData += writeSize;
        remaining -= writeSize;
    }

    return CY_SMIF_SUCCESS;
}

#endif //ESTEC_FALSH_QSPI_ENABLE

