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
#include "main_config.h"
#include "mcu_i2s_dw.h"

/*
    This example transmits .wav file to external audio DAC (AIC261)
    via I2S protocol.
    Please note this example supports only 16 bit length PCM data.
    TVII is master, and the AIC261 is slave. Sampling rate, 
    monaural/stereo will be determined by wav header.
    When internal clock is used, please connect I2S_MCLK to MCLK input
    port of the AIC261. When external clock is used please connect
    external crystal output to I2S_IF of TVII and to MCLK input port
    of the AIC261.
    - Operation Description
    I2S HW will trigger DW transmission. The DW will then transmit
    sound data to I2S FIFO. Any CPU task is not required.
*/


//#define ESTEC_PCM_32BIT_SUPPORT		(1) //KMS251110_3 : HKMC mandatory is 16bit PCM MONO(Sampling rate : 48KHz(Guide voice) or 24KHz(VESS))

#ifdef ESTEC_MCU_I2S_DW_ENABLE

/******************************************************************************/
/*                      AUDIOSS Port (I2S1) and SCB                           */
/******************************************************************************/

#define CY_AUDIOSS_TYPE                         I2S0

#define CY_AUDIOSS_MCLK_PORT                    GPIO_PRT11
#define CY_AUDIOSS_MCLK_PIN                     0
#define CY_AUDIOSS_MCLK_PIN_MUX                 P11_0_AUDIOSS0_MCLK

#define CY_AUDIOSS_TX_SCK_PORT                  GPIO_PRT11
#define CY_AUDIOSS_TX_SCK_PIN                   1
#define CY_AUDIOSS_TX_SCK_PIN_MUX               P11_1_AUDIOSS0_TX_SCK

#define CY_AUDIOSS_TX_WS_PORT                   GPIO_PRT11
#define CY_AUDIOSS_TX_WS_PIN                    2
#define CY_AUDIOSS_TX_WS_PIN_MUX                P11_2_AUDIOSS0_TX_WS

#define CY_AUDIOSS_TX_SDO_PORT                  GPIO_PRT12
#define CY_AUDIOSS_TX_SDO_PIN                   0
#define CY_AUDIOSS_TX_SDO_PIN_MUX               P12_0_AUDIOSS0_TX_SDO

/* AUDIOSS-DW */
#define CY_AUDIOSS_TX_DW                        DW1
#define CY_AUDIOSS_TX_DW_CH                     52
#define CY_AUDIOSS_TX_DW_TRIG                   TRIG_IN_1TO1_5_I2S_TX_TO_PDMA10

#define CY_AUDIOSS_RX_DW                        DW1
#define CY_AUDIOSS_RX_DW_CH                     53
#define CY_AUDIOSS_RX_DW_TRIG                   TRIG_IN_1TO1_5_I2S_RX_TO_PDMA10

/* Device Specific Definition  */
#define AUDIO_I2S_DW            (CY_AUDIOSS_TX_DW)
#define AUDIO_I2S_TX_DW_LOG_CH  (CY_AUDIOSS_TX_DW_CH)
#define AUDIO_I2S_TX_TO_DW_TRIG (CY_AUDIOSS_TX_DW_TRIG)

/* User setting for clock and  */
//------------------------------------------------------------------------------
#define I2S_INTERNAL_CLOCK          (0ul)
#define I2S_EXTERNAL_CLOCK          (1ul)
#define CY_I2S_USE_CLK              I2S_INTERNAL_CLOCK

#if(CY_I2S_USE_CLK == I2S_INTERNAL_CLOCK)
  #define AUDIO_SOURCE_FREQ_IN_HZ (196608000ul) // Assumed this value has been configured in system initialization.
#else // I2S_EXTERNAL_CLOCK
  #define AUDIO_SOURCE_FREQ_IN_HZ (24576000ul) // Frequency of External Crystal
#endif
#define AUDIO_CONST_DIV_NUM     (8ul)         // Note: this is specified by HW IP
#ifdef ESTEC_PCM_32BIT_SUPPORT
#define PCM_DATA_WIDTH          (32ul)        // this example only support 32 bit PCM
#else
#define PCM_DATA_WIDTH          (16ul)        // this example only support 16 bit PCM
#endif
#define I2S_CHANNEL_NUM         (2ul)         // fixed 2 channel in I2S mode

#define AUDIO_MCLK_DIV_CONFIG (CY_I2S_MCLK_DIV_8) // User defined value
#define AUDIO_MCLK_DIV_NUM    (1ul << AUDIO_MCLK_DIV_CONFIG)
#if(CY_I2S_USE_CLK == I2S_INTERNAL_CLOCK)
  #define AUDIO_MCLK_CLOCK_HZ   (AUDIO_SOURCE_FREQ_IN_HZ / AUDIO_MCLK_DIV_NUM)
#else
  #define AUDIO_MCLK_CLOCK_HZ   AUDIO_SOURCE_FREQ_IN_HZ // Crystal clock will go directory MCLK port
#endif

#pragma pack(1)
typedef struct {
    uint32_t riff;         // offset: 0,  size: 4, Indicates type RIFF value = 'RIFF'
    uint32_t size;         // offset: 4,  size: 4, file size - 8
    uint32_t type;         // offset: 8,  size: 4, = 'WAVE'
    uint32_t id;           // offset: 12, size: 4, Format chunk marker. Includes trailing null
    uint32_t chanksize;    // offset: 16, size: 4, chunk data size (16)
    uint16_t format;       // offset: 20, size: 2, Type of format (1 is PCM) - 2 byte integer
    uint16_t channels;     // offset: 22, size: 2, Number of Channels - 2 byte integer
    uint32_t samplerate;   // offset: 24, size: 4, Sample Rate
    uint32_t bytepersec;   // offset: 28, size: 4, (Sample Rate * BitsPerSample * Channels) / 8
    uint16_t blockalign;   // offset: 30, size: 2, data block size
    uint16_t bitswidth;    // offset: 32, size: 2, Bits per sample
    uint32_t data;         // offset: 36, size: 4, "data"
    uint32_t bytesize;     // offset: 40, size: 4, Size of the data section
} stc_wav_header;
#define WAV_HEADER_SIZE (sizeof(stc_wav_header))
#pragma pack()

// Sound Data
const uint8_t g_Wav16Data[] =
{
    #include "wav_16bit_data.inc"
};
#define I2S_SOUND_SIZE_IN_BYTE ((sizeof(g_Wav16Data) / sizeof(g_Wav16Data[0])) - (sizeof(stc_wav_header)))
#define I2S_SOUND_PCM_NUMBER   (I2S_SOUND_SIZE_IN_BYTE / 2ul)
uint16_t*       gp_SoundData = (uint16_t*)&g_Wav16Data[WAV_HEADER_SIZE];
stc_wav_header* gp_WavHeader = (stc_wav_header*)g_Wav16Data;
uint32_t        g_PcmNumber; // will be updated in runtime

/* I2S-Tx and I2S-Rx interface configuration */
//------------------------------------------------------------------------------
#define I2S_TX_FIFO_TH_NUM   (CY_I2S_TX_FIFO_NUM/2ul)
#define I2S_TX_FIFO_REST_NUM (CY_I2S_TX_FIFO_NUM - I2S_TX_FIFO_TH_NUM)

cy_stc_i2s_config_clk_t g_i2s_clk_config =
{
    .clkDiv  = 0u, // will be modified in runtime
    .extClk  = (bool)CY_I2S_USE_CLK,
    .mclkDiv = AUDIO_MCLK_DIV_CONFIG,
    .mclkEn  = true,
};

cy_stc_i2s_config_tx_t g_i2s_tx_config =
{
    .txEnabled          = true,
    .txDmaTrigger       = true,
    .txMasterMode       = true,
    .txAlignment        = CY_I2S_I2S_MODE,
    .txWsPulseWidth     = CY_I2S_WS_ONE_CHANNEL_LENGTH, // Not be cared in I2S mode
    .txWatchdogEnable   = false,
    .txWatchdogValue    = 0xFFFFFFFFul,
    .txSdoLatchingTime  = false,
    .txSckoInversion    = false,
    .txSckiInversion    = false,
    .txChannels         = 0ul,                          // Not be cared in I2S mode
#ifdef FDA806D_AMP_ENABLE
    .txChannelLength    = CY_I2S_LEN32, //KMS251110_1 : FDA803D shoud be 32bit.
#else
    .txChannelLength    = CY_I2S_LEN16,
#endif
#ifdef ESTEC_PCM_32BIT_SUPPORT
    .txWordLength       = CY_I2S_LEN32, //KMS251110_2 : 32bit (must be less or equal to txChannelLength. )
#else
    .txWordLength       = CY_I2S_LEN16, //KMS251110_2 : 16bit
#endif
    .txOverheadValue    = CY_I2S_OVHDATA_ZERO,
    .txFifoTriggerLevel = I2S_TX_FIFO_TH_NUM,
};

cy_stc_gpio_pin_prt_config_t g_i2s_pin_config[] =
{
//  {                    portReg,                    pinNum, outVal,                driveMode,                         hsiom, intEdge, intMask, vtrip, slewRate, driveSel },
    {     CY_AUDIOSS_TX_SCK_PORT,     CY_AUDIOSS_TX_SCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,     CY_AUDIOSS_TX_SCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {      CY_AUDIOSS_TX_WS_PORT,      CY_AUDIOSS_TX_WS_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,      CY_AUDIOSS_TX_WS_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {     CY_AUDIOSS_TX_SDO_PORT,     CY_AUDIOSS_TX_SDO_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,     CY_AUDIOSS_TX_SDO_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#if (CY_I2S_USE_CLK == I2S_INTERNAL_CLOCK)
    {       CY_AUDIOSS_MCLK_PORT,       CY_AUDIOSS_MCLK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,       CY_AUDIOSS_MCLK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#else
    { CY_AUDIOSS_CLK_I2S_IF_PORT, CY_AUDIOSS_CLK_I2S_IF_PIN,    0ul,         CY_GPIO_DM_HIGHZ, CY_AUDIOSS_CLK_I2S_IF_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#endif
};

#define I2S_PORT_NUM (sizeof(g_i2s_pin_config)/sizeof(g_i2s_pin_config[0]))

/* I2S dedicated DW */
#ifdef ESTEC_PCM_32BIT_SUPPORT
#define RESERVED_DESCRIPTOR_NUM (1024)
#else
#define RESERVED_DESCRIPTOR_NUM (512)
#endif
static cy_stc_pdma_descr_t g_stcDescr[RESERVED_DESCRIPTOR_NUM];

static cy_stc_pdma_chnl_config_t   chnlConfig =
{
    .PDMA_Descriptor = g_stcDescr, // will be updated in runtime
    .preemptable     = 0ul,
    .priority        = 0ul,
    .enable          = 1ul,  // enabled after initialization
};

static cy_stc_pdma_descr_config_t  stcDmaDescrConfig =
{
    .deact          = 0ul,
    .intrType       = CY_PDMA_INTR_DESCR_CMPLT,
    .trigoutType    = CY_PDMA_TRIGOUT_DESCR_CMPLT,
    .chStateAtCmplt = CY_PDMA_CH_ENABLED,
    .triginType     = CY_PDMA_TRIGIN_1ELEMENT,
#ifdef ESTEC_PCM_32BIT_SUPPORT
	.dataSize       = CY_PDMA_WORD,
#else
    .dataSize       = CY_PDMA_HALFWORD,
#endif
    .srcTxfrSize    = 0ul,
    .destTxfrSize   = 1ul, // 32bit
    .descrType      = CY_PDMA_2D_TRANSFER,
    .srcAddr        = (uint32_t *)&g_Wav16Data[WAV_HEADER_SIZE],
    .destAddr       = (uint32_t *)&CY_AUDIOSS_TYPE->unTX_FIFO_WR.u32Register,
    .srcXincr       = 0ul, // will be updated in runtime
    .destXincr      = 0ul,
    .xCount         = 0ul, // will be updated in runtime
    .srcYincr       = 0ul, // will be updated in runtime
    .destYincr      = 0ul,
    .yCount         = 0ul, // will be updated in runtime
    .descrNext      = NULL, // will be updated in runtime
};

static void AudioDataIntegrityCheck(void);
static void PrepareDwDescriptorMono(void);
static void PrepareDwDescriptorStereo(void);

void I2S_Init(void)
{
    // Check wav file data integrity
    AudioDataIntegrityCheck();

    /**************************************/
    /* Port setting for I2S communication */
    /**************************************/
    Cy_GPIO_Multi_Pin_Init(g_i2s_pin_config, I2S_PORT_NUM);

    /*******************************************/
    /*    Initialization DW dedicating to I2S  */
    /*******************************************/
    if(gp_WavHeader->channels == 2)
    {
        PrepareDwDescriptorStereo();
    }
    else
    {
        PrepareDwDescriptorMono();
    }

    Cy_PDMA_Disable(AUDIO_I2S_DW);
    Cy_PDMA_Chnl_DeInit(AUDIO_I2S_DW, AUDIO_I2S_TX_DW_LOG_CH);
    Cy_PDMA_Chnl_Init(AUDIO_I2S_DW, AUDIO_I2S_TX_DW_LOG_CH, &chnlConfig);
    Cy_PDMA_Chnl_Enable(AUDIO_I2S_DW, AUDIO_I2S_TX_DW_LOG_CH);
    Cy_PDMA_Chnl_SetInterruptMask(DW1, AUDIO_I2S_TX_DW_LOG_CH);
    Cy_PDMA_Enable(AUDIO_I2S_DW);
    /*    Setting Trigger Mux  */
    Cy_TrigMux_Connect1To1(AUDIO_I2S_TX_TO_DW_TRIG, 0ul, TRIGGER_TYPE_LEVEL, 0ul);

    /********************************************/
    /*          initialization for I2S          */
    /********************************************/
    // De-Initialize I2S
    Cy_I2S_DeInit(CY_AUDIOSS_TYPE);

    // Flush FIFO of I2S 
    Cy_I2S_ClearTxFifo(CY_AUDIOSS_TYPE);
    Cy_I2S_ClearRxFifo(CY_AUDIOSS_TYPE);

    // Calculate clock divider number as per sound data format
#ifdef FDA806D_AMP_ENABLE //KMS251111_1 : FDA806D BCK shoud be 64fs.
    g_i2s_clk_config.clkDiv = AUDIO_SOURCE_FREQ_IN_HZ / (AUDIO_CONST_DIV_NUM * gp_WavHeader->samplerate * 32 * I2S_CHANNEL_NUM);
#else
    g_i2s_clk_config.clkDiv = AUDIO_SOURCE_FREQ_IN_HZ / (AUDIO_CONST_DIV_NUM * gp_WavHeader->samplerate * PCM_DATA_WIDTH * I2S_CHANNEL_NUM);
    CY_ASSERT(AUDIO_SOURCE_FREQ_IN_HZ % (AUDIO_CONST_DIV_NUM * gp_WavHeader->samplerate * PCM_DATA_WIDTH) == 0ul);
#endif
    // Initialize I2S Clock settings
    CY_ASSERT(Cy_I2S_InitClock(CY_AUDIOSS_TYPE, &g_i2s_clk_config) == CY_I2S_SUCCESS);

    // Initialize I2S Tx settings
    CY_ASSERT(Cy_I2S_InitTx(CY_AUDIOSS_TYPE, &g_i2s_tx_config) == CY_I2S_SUCCESS);

    /* Start I2S Tx */
    Cy_I2S_EnableTx(CY_AUDIOSS_TYPE);
}

#define DW_DEST_COUNT_BIT_NUM (8ul)
#define DW_DEST_COUNT_MAX     (1ul << DW_DEST_COUNT_BIT_NUM)
#define DW_DEST_COUNT_XY_MAX  (1ul << (DW_DEST_COUNT_BIT_NUM*2))
#define DW_DEST_COUNT_MASK    (DW_DEST_COUNT_MAX - 1ul)
#define DW_DEST_COUNT_XY_MASK (DW_DEST_COUNT_XY_MAX - 1ul)
static void PrepareDwDescriptorStereo(void)
{
    stcDmaDescrConfig.srcXincr = 1ul;

    uint32_t restDataSize = I2S_SOUND_PCM_NUMBER;
    uint32_t counter = 0ul;
    while(restDataSize > 0)
    {
        uint32_t startPoint = I2S_SOUND_PCM_NUMBER - restDataSize;
        stcDmaDescrConfig.srcAddr = &gp_SoundData[startPoint];
        if(restDataSize > DW_DEST_COUNT_XY_MAX)
        {
            stcDmaDescrConfig.xCount = DW_DEST_COUNT_MAX;
            stcDmaDescrConfig.yCount = DW_DEST_COUNT_MAX;
            stcDmaDescrConfig.srcYincr = DW_DEST_COUNT_MAX;
            stcDmaDescrConfig.descrNext = &g_stcDescr[counter+1];
            restDataSize -= DW_DEST_COUNT_XY_MAX;
        }
        else if(restDataSize > DW_DEST_COUNT_MAX)
        {
            stcDmaDescrConfig.xCount = DW_DEST_COUNT_MAX;
            stcDmaDescrConfig.yCount = restDataSize >> DW_DEST_COUNT_BIT_NUM;
            stcDmaDescrConfig.srcYincr = DW_DEST_COUNT_MAX;
            stcDmaDescrConfig.descrNext = &g_stcDescr[counter+1];
            restDataSize -= DW_DEST_COUNT_MAX * stcDmaDescrConfig.yCount;
        }
        else
        {
            stcDmaDescrConfig.xCount = restDataSize;
            stcDmaDescrConfig.yCount = 1;
            stcDmaDescrConfig.srcYincr = restDataSize;
            stcDmaDescrConfig.descrNext = &g_stcDescr[0];
            restDataSize = 0ul;
        }

        Cy_PDMA_Descr_Init(&g_stcDescr[counter], &stcDmaDescrConfig);
        counter++;
    }
}

static void PrepareDwDescriptorMono(void)
{
    uint32_t descNum = (I2S_SOUND_PCM_NUMBER >> DW_DEST_COUNT_BIT_NUM)  + 1ul;
    CY_ASSERT(descNum < RESERVED_DESCRIPTOR_NUM);
    uint32_t restPcmNum = (I2S_SOUND_PCM_NUMBER & DW_DEST_COUNT_MASK);

    stcDmaDescrConfig.srcXincr = 0ul;
    stcDmaDescrConfig.xCount   = 2ul; // mono L and R
    stcDmaDescrConfig.srcYincr = 1ul;

    for(uint32_t i = 0ul; i < descNum; i++)
    {
        stcDmaDescrConfig.srcAddr = &gp_SoundData[DW_DEST_COUNT_MAX*i];
        if(i == (descNum - 1ul)) // last descriptor
        {
            stcDmaDescrConfig.yCount = restPcmNum;
            stcDmaDescrConfig.descrNext = &g_stcDescr[0];
        }
        else
        {
            stcDmaDescrConfig.yCount = DW_DEST_COUNT_MAX;
            stcDmaDescrConfig.descrNext = &g_stcDescr[i+1];
        }
        Cy_PDMA_Descr_Init(&g_stcDescr[i],&stcDmaDescrConfig);
    }
}

static void AudioDataIntegrityCheck(void)
{
    CY_ASSERT(memcmp((void*)&gp_WavHeader->riff, "RIFF", 4) == 0);
    CY_ASSERT(memcmp((void*)&gp_WavHeader->type, "WAVE", 4) == 0);
    CY_ASSERT(gp_WavHeader->format == 1/*PCM*/);
    CY_ASSERT((gp_WavHeader->channels == 1) || (gp_WavHeader->channels == 2));
    CY_ASSERT(gp_WavHeader->bitswidth == PCM_DATA_WIDTH);
}

#endif //ESTEC_MCU_I2S_DW_ENABLE

/* [] END OF FILE */
