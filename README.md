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
