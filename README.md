# Revision History

## 2025-09-01
  - Made AVAS Test Sample Codes.
  - Added CAN PORT. //KMS250901_1
  - Deleted I2S Code for Test. Please refer to "MCU_I2S_SUPPORT" macro. //KMS250901_2
  
## 2025-09-04
  - Added SPI code to control TCAN1445. //KMS250904_1
  - Deleted M7_1 related codes because we don't use it under CYT3BB. //KMS250904_2

## 2025-09-05
  - Deleted CY_SYSTEM_WCO_ENABLE define because we don't use external 48.xxKHz Crystal. //KMS250905
  - Changed ECO setting value. //KMS250905_1

## 2025-09-08
  - Deleted NON_ISO_OPERATION define due to CAN working. //KMS250908

## 2025-09-09
  - Changed the CY_SYS_VCCD_SOURCE to CY_SYS_VCCD_PASS_TR because we don't use PMIC. //KMS250909
  - Added "ESTEC_BOARD" pre-define in preprocessor in C/C++compiler in Option for node "cm0plus" in compiler option.

## 2025-09-25
  - Added EVK_TEST define. //KMS250925_1

## 2025-11-04
  - Implemented I2C and GPIO features to contorl FDA806D

## 2025-11-05
  - Separated AMP code and CAN code. It's working in cm0plus case.
  - Added mute function.
  
## 2025-11-10
  - Implemented I2S feature and AVAS feature. //KMS251110_1 / KMS251110_2
  - Added 32bit PCM Mono but 16bit PCM Mono is mandatory. //KMS251110_3
  - Moved these codes(CAN related defines) to TCAN1145_can.c. //KMS251110_4

## 2025-11-11
  - FDA806D BCK shoud be 64fs. This change is to follow HKMC spec(AVAS 24KHz / etc 48KHz). //KMS251111_1
  
## 2025-11-14
  - Implemented external Flash(W25Q32) control for logging. Please refer to w25q32_scb.c & w25q32_scb.h.
    # Spi_SCB3_Init() funciton has total 6 tests which are releated with flash control.
      1. Chip Erase(4MB) & Noraml Read Byte
      2. Byte Write(Write 256 byte one by one from address of 0 to 255) & Noraml Read byte
      3. Erase a Block 1(64KB) & Noraml Read 256 byte
      4. Write a page(Max 256 byte /It's possible from 1byte to 256byte) & Fast Read 256 byte
      5. Erase(32KB) & Noraml Read 256 byte
      6. Erase a sector(4KB) & Noraml Read 256 byte
  - Added external Flash contorl for audio source. Please refer to sc park's codes.
  