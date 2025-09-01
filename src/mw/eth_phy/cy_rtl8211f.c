/***************************************************************************//**
* \file cy_rtl8211f.c
*
* \brief Driver for Ethernet PHY RTL8211F
* 
*
********************************************************************************
* \copyright
* Copyright 2016-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"

/* Only applicable for the lite kit */
#ifdef CY_327BGA_EVKLITE_rev_a

#include "cy_rtl8211f.h"

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/


/*****************************************************************************
* Local type definitions ('typedef')
*****************************************************************************/


/*****************************************************************************
* Local function prototypes ('static')                                                                            
*****************************************************************************/


/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/


/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/
/*******************************************************************************
* Function Name: Cy_Rtl8211f_Reset
****************************************************************************//**
*
* \brief Reset RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
*******************************************************************************/
void Cy_Rtl8211f_Reset(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_bmcr_t   bmcr;

    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    if(bmcr.u16 == 0u)
    {
        bmcr.f.u1RESET = 1u;
        Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_BMCR, bmcr.u16);
    }
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_IsResetDone
****************************************************************************//**
*
* \brief Checks if the reset operation is completed or not.
*
* \param handle
* The accessing object to be initialized.
*
* \return bool
* Returns if the reset is done or not. 
*
*******************************************************************************/
bool Cy_Rtl8211f_IsResetDone(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_bmcr_t   bmcr;
    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    if(bmcr.f.u1RESET == 0u)
    {
        return true;
    }
    else 
    {
        return false;
    }
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_Suspend
****************************************************************************//**
*
* \brief Suspend RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
*******************************************************************************/
void Cy_Rtl8211f_Suspend(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_bmcr_t   bmcr;
    
    /* Read the register */
    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    
    /* Update and write the register */
    bmcr.f.u1POWER_DOWN = 1u;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_BMCR, bmcr.u16);
    
    /* Wait until PHY is in power down mode */
    do {
        bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    } while(bmcr.f.u1POWER_DOWN != 1u);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_Resume
****************************************************************************//**
*
* \brief Resume RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
*******************************************************************************/
void Cy_Rtl8211f_Resume(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_bmcr_t   bmcr;
    
    /* Read the register */
    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    
    /* Update and write the register */
    bmcr.f.u1POWER_DOWN = 0u;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_BMCR, bmcr.u16);
    
    /* Wait until PHY is in normal mode */
    do {
        bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    } while(bmcr.f.u1POWER_DOWN != 0u);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_GetPhyID
****************************************************************************//**
*
* \brief Check if the RTL8211F ethernet PHY ID Matches.
*
* \param handle
* The accessing object to be initialized.
*
* \return bool
* Returns if the PHY ID is correct or not. 
*
*******************************************************************************/
bool Cy_Rtl8211f_GetPhyID(cy_stc_rtl8211f_t* handle, uint32_t phy_id, uint32_t phy_id_msk)
{
    cy_un_rtl8211f_phyidr1_t  phyid1;
    cy_un_rtl8211f_phyidr2_t  phyid2;
    phyid1.u16 = 0u;
    phyid2.u16 = 0u;
    phyid1.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYIDR1);
    phyid2.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYIDR2);
    
    if ((((phyid1.u16<< 16u) | phyid2.u16) & phy_id_msk) != phy_id)
    {
        return false;
    }
    return true;
}   

/*******************************************************************************
* Function Name: Cy_Rtl8211f_IsAutonegotiationEnabled
****************************************************************************//**
*
* \brief Checks if Auto Negotiation is Enabled in RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
* \return bool
* Returns if Auto Negotiation is enabled or not.  
*******************************************************************************/
bool Cy_Rtl8211f_IsAutonegotiationEnabled(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_bmcr_t  bmcr;
    bmcr.u16 = 0u;
    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    return (((bmcr.f.u1AUTO_NEGOTIATION_ENABLE) != 0u) ? true : false);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_IsAutonegotiationDone
****************************************************************************//**
*
* \brief Checks if Auto Negotiation is done in RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
* \return bool
* Returns if Auto Negotiation is done or not.  
*******************************************************************************/
bool Cy_Rtl8211f_IsAutonegotiationDone(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_bmsr_t  bmsr;
    bmsr.u16 = 0u;
    bmsr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMSR);
    return (((bmsr.f.u1AUTO_NEGOTIATION_COMPLETE) != 0u) ? true : false);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_IsRemoteReceiverOk
****************************************************************************//**
*
* \brief Checks if Remote receiver status in RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
* \return bool
* Returns the status as OK or not.  
*******************************************************************************/
bool Cy_Rtl8211f_IsRemoteReceiverOk(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_gbsr_t  gbsr;
    gbsr.u16 = 0u;
    gbsr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_GBSR);
    return (((gbsr.f.u1MASTER_SLAVE_CONFIGURATION_FAULT | gbsr.f.u1REMOTE_RECEIVER_STATUS | gbsr.f.u1LOCAL_RECEIVER_STATUS)) ? true : false);
}


/*******************************************************************************
* Function Name: Cy_Rtl8211f_WriteMmdIndirect
****************************************************************************//**
*
* \brief Funtion to write to the MMD in RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
* \param dev_addr
* The device address to which it needs to be written
*
* \param mmd_addr
* The memory manageable device that is to be written to. 
*
* \param data
* The data to be written to the MMD.
*
*******************************************************************************/
void Cy_Rtl8211f_WriteMmdIndirect(cy_stc_rtl8211f_t* handle, uint8_t dev_addr, uint16_t mmd_addr, uint16_t data)
{
    cy_un_rtl8211f_macr_t macr;
    cy_un_rtl8211f_maddr_t maddr;
    
    macr.u16 = 0u;
    macr.f.u5DEVAD = dev_addr;
    macr.f.u2FUNCTION = 0x00;
    maddr.f.u16ADDRESS_DATA = mmd_addr;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MACR, macr.u16);
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MADDR, maddr.u16);
    
    macr.f.u5DEVAD = dev_addr;
    macr.f.u2FUNCTION = 0x01;
    maddr.f.u16ADDRESS_DATA = data;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MACR, macr.u16);
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MADDR, maddr.u16);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_ReadMmdIndirect
****************************************************************************//**
*
* \brief Funtion to read from the MMD in RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
* \param dev_addr
* The device address to which it needs to be written to.
*
* \param mmd_addr
* The memory manageable device that is to be written to. 
*
* \param data
* The data in the MMD.
*
* \return uint16_t
* Returns the data in the MMD device. 
* 
*******************************************************************************/
uint16_t Cy_Rtl8211f_ReadMmdIndirect(cy_stc_rtl8211f_t* handle, uint8_t dev_addr, uint16_t mmd_addr, uint16_t data)
{
    cy_un_rtl8211f_macr_t macr;
    cy_un_rtl8211f_maddr_t maddr;
    
    macr.u16 = 0u;
    macr.f.u5DEVAD = dev_addr;
    macr.f.u2FUNCTION = 0x00;
    maddr.f.u16ADDRESS_DATA = mmd_addr;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MACR, macr.u16);
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MADDR, maddr.u16);
    
    macr.f.u5DEVAD = dev_addr;
    macr.f.u2FUNCTION = 0x01;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_MACR, macr.u16);
    data = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_MADDR);
    return data;
}


/*******************************************************************************
* Function Name: Cy_Rtl8211f_ClearEeeAdv
****************************************************************************//**
*
* \brief Clears the EEE Advertisement in RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
*******************************************************************************/
void Cy_Rtl8211f_ClearEeeAdv(cy_stc_rtl8211f_t* handle)
{
    Cy_Rtl8211f_WriteMmdIndirect(handle, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_HandleInit
****************************************************************************//**
*
* \brief Initialize an object which accesses RTL8211F ethernet PHY.
*
* \param pEth
* Pointer to top of ethernet IP register area.
*
* \param phyAddr
* Address of the ether PHY
*
* \param handle
* The accessing object to be initialized.
*
*******************************************************************************/
void Cy_Rtl8211f_HandleInit(volatile stc_ETH_t* pEth, uint8_t phyAddr, cy_stc_rtl8211f_t* handle)
{
    CY_ASSERT(pEth != NULL);

    handle->pEth    = pEth;
    handle->phyAddr = phyAddr;
}


/*******************************************************************************
* Function Name: Cy_Rtl8211f_Init
****************************************************************************//**
*
* \brief Initialize and Configure the RTL8211F ethernet PHY.
*
* \param handle
* The accessing object to be initialized.
*
* \return uint32_t
* Returns the status of configuration. Refer to \ref cy_en_rtl8211f_status_t  
*
*******************************************************************************/
uint32_t Cy_Rtl8211f_Init(cy_stc_rtl8211f_t* handle ,cy_stc_rtl8211f_phy_cfg_t * phyCfg)
{
    cy_un_rtl8211f_pagsr_t  pagsr = {0};
    cy_un_rtl8211f_phycr2_t phycr2 = {0};
    cy_un_rtl8211f_bmcr_t   bmcr = {0};
    cy_un_rtl8211f_anar_t   anar = {0};
    cy_un_rtl8211f_gbcr_t   gbcr = {0};
    //cy_un_rtl8211f_bmsr_t   bmsr = {0};
    uint32_t retries;
    uint16_t regval;
    volatile uint16_t phy2Val = 0u;

    /* Check the PHY ID */
    retries = 0xffffffff;
    while ((Cy_Rtl8211f_GetPhyID(handle ,RTL8211F_PHY_ID, RTL8211F_PHY_ID_MSK) != true) && --retries);
    if (retries == 0u)
    {
        return ETH_PHY_STATUS_ERROR_ID;
    }
    
    /* Check the status */
    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    //bmsr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMSR);

    /* Update to the PHY register page */
    pagsr.u16 = 0xa43;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);
    phycr2.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYCR2);

    /* Check for the clock requirements */
    if (phyCfg->clkout == ETH_PHY_CONFIG_ENABLE_CLKOUT_125MHZ)
    {
        /* enable clock out 125MHz */
        phycr2.f.u1CLKOUT_EN = 1u;
        phycr2.f.u1CLKOUT_FREQ_SEL = 1u;
    }
    else if(phyCfg->clkout == ETH_PHY_CONFIG_ENABLE_CLKOUT_25MHZ)
    {
        /* enable 25Mhz clock */
        phycr2.f.u1CLKOUT_EN = 1u;
        phycr2.f.u1CLKOUT_FREQ_SEL = 0u;
    }
    else
    {
        phycr2.f.u1CLKOUT_EN = 0u;
    }

    /* Disable EEE mode */
    phycr2.f.u1PHY_EEE_EN = 0u;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PHYCR2, phycr2.u16);
    phycr2.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYCR2);
    
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    /* Reset and suspend the module */
    Cy_Rtl8211f_Reset(handle);
    while (Cy_Rtl8211f_IsResetDone(handle) != true);
    Cy_Rtl8211f_Suspend(handle);

    /* enable TX-delay for rgmii-id and rgmii-txid, otherwise disable it */
    pagsr.u16 = 0xd08;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);
    regval = Cy_Rtl8211f_ReadReg(handle, (cy_en_rtl8211f_reg_info_t) 0x11);
    
    if ((phyCfg->ifMode == ETH_PHY_CONFIG_INTERFACE_RGMII_ID) || (phyCfg->ifMode == ETH_PHY_CONFIG_INTERFACE_RGMII_TXID))
    {
        regval |= ETH_PHY_TX_DELAY_EN;
    }
    else
    {   
        regval &= ~ETH_PHY_TX_DELAY_EN;
    }
    Cy_Rtl8211f_WriteReg(handle, (cy_en_rtl8211f_reg_info_t) 0x11, regval);
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);
    
    /* enable RX-delay for rgmii-id and rgmii-rxid, otherwise disable it */
    pagsr.u16 = 0xd08;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);
    regval = Cy_Rtl8211f_ReadReg(handle, (cy_en_rtl8211f_reg_info_t) 0x15);
    
    if ((phyCfg->ifMode == ETH_PHY_CONFIG_INTERFACE_RGMII_ID) || (phyCfg->ifMode == ETH_PHY_CONFIG_INTERFACE_RGMII_RXID))
    {
        regval |= ETH_PHY_RX_DELAY_EN;
    }
    else
    {   
        regval &= ~ETH_PHY_RX_DELAY_EN;
    }
    Cy_Rtl8211f_WriteReg(handle, (cy_en_rtl8211f_reg_info_t) 0x15, regval);
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);
    
    /* Check for the auto negotiation */
    bmcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_BMCR);
    if ((phyCfg->enableAutoNeg) != 0u)
    {
        /* Clear the EEE type advertisement */
        Cy_Rtl8211f_ClearEeeAdv(handle);

        /* Check for the configured advertising mode */
        if ((phyCfg->advMode & ETH_PHY_CONFIG_ADVERTISING_Msk) != 0u)
        {
            /* Read the MII advertising registers */
            anar.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_ANAR);
            
            /* Clear all the advertising mode */
            anar.u16 &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);
            
            /* Advertise only requested modes (can be OR'ed) if requested in config */
            anar.u16 |= (phyCfg->advMode & ETH_PHY_CONFIG_ADVERTISE_10_100_Msk) * ADVERTISE_10HALF;
            Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_ANAR, anar.u16);
            
            /* Read the MII1000 configuration registers */
            gbcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_GBCR);
            if ((phyCfg->speedMode & ETH_PHY_CONFIG_LINK_MODE_1000BASET_FULL) == 0u)
            {
                gbcr.f.u1_1000BASE_T_FULL_DUPLEX = 0u;
            }
            else
            {
                gbcr.f.u1_1000BASE_T_FULL_DUPLEX = 1u;
            }

            /* Update the MII1000 configuration registers */
            Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_GBCR, gbcr.u16);
        }
        
        /* not really needed but added to be complete, since auto negotation is default */
        bmcr.f.u1AUTO_NEGOTIATION_ENABLE = 1u;
        Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_BMCR, bmcr.u16);
    }
    else
    {
        /* Reset the speed mode and disable the auto negotiation */
        bmcr.f.u1AUTO_NEGOTIATION_ENABLE = 0u;
        bmcr.f.u1SPEED_SELECTION_LSB = 0u;
        bmcr.f.u1SPEED_SELECTION_MSB = 0u;
      
        switch (phyCfg->speedMode)
        {
            case ETH_PHY_CONFIG_LINK_MODE_10BASET_HALF:
                break;
            case ETH_PHY_CONFIG_LINK_MODE_10BASET_FULL:
                bmcr.f.u1DUPLEX_MODE = 1u;
                break;
            case ETH_PHY_CONFIG_LINK_MODE_100BASET_HALF:
                bmcr.f.u1SPEED_SELECTION_LSB = 1u;
                break;
            case ETH_PHY_CONFIG_LINK_MODE_100BASET_FULL:
                bmcr.f.u1SPEED_SELECTION_LSB = 1u;
                bmcr.f.u1DUPLEX_MODE = 1u;
                break;
            case ETH_PHY_CONFIG_LINK_MODE_1000BASET_FULL:
                bmcr.f.u1SPEED_SELECTION_MSB = 1u;
                bmcr.f.u1DUPLEX_MODE = 1u;
                break;
            default:
                return ETH_PHY_STATUS_ERROR_IF;
                //break;
        }

        /* Check if loopback is enabled */
        if (phyCfg->enableLoopback)
        {
            bmcr.f.u1LOOPBACK = 1u;
        }

        /* Update the mode control register */
        Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_BMCR, bmcr.u16);
    }
    
    /* Software reset is asserted, Restart AN or power mode transition */
    //Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_BMCR, bmcr.u16);
    Cy_Rtl8211f_Resume(handle);
    
    return ETH_PHY_STATUS_OK;
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_ReadReg
****************************************************************************//**
*
* \brief Read normal register of the RTL8211F ethernet PHY.
*
* \param reg
* The register information \ref cy_en_rtl8211f_reg_info_t
*
* \param handle
* The accessing object
*
* \return
* read value
*
*******************************************************************************/
uint16_t Cy_Rtl8211f_ReadReg(cy_stc_rtl8211f_t* handle, cy_en_rtl8211f_reg_info_t reg)
{
    CY_ASSERT(handle->pEth != NULL);

    uint16_t addr = reg;
    return Cy_EthIf_PhyRegRead(handle->pEth, addr, handle->phyAddr);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_WriteReg
****************************************************************************//**
*
* \brief Write normal register of the RTL8211F ethernet PHY.
*
* \param reg
* The register information \ref cy_en_rtl8211f_reg_info_t
*
* \param data
* The value to be written to the register.
*
* \param handle
* The accessing object.
*
*******************************************************************************/
void Cy_Rtl8211f_WriteReg(cy_stc_rtl8211f_t* handle, cy_en_rtl8211f_reg_info_t reg, uint16_t data)
{
    CY_ASSERT(handle->pEth != NULL);

    uint16_t addr = reg;
    Cy_EthIf_PhyRegWrite(handle->pEth, addr, data, handle->phyAddr);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_GetLinkStatus
****************************************************************************//**
*
* \brief Function checks and returns the link status from a RTL8211F register
*
* \param handle
* The accessing object.
* 
* \return bool
* Returns if the Link is up(true) or not(false)
*
*******************************************************************************/
bool Cy_Rtl8211f_GetLinkStatus(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_physr_t physr;
    cy_un_rtl8211f_pagsr_t pagsr;

    pagsr.f.u12PAGE_SEL = 0xa43;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    physr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYSR);
    
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    return ((physr.f.u1LINK_STATUS == 1) ? true : false);
}


/*******************************************************************************
* Function Name: Cy_Rtl8211f_GetLinkSpeed
****************************************************************************//**
*
* \brief Function checks and returns the link speed from a RTL8211F register
*
* \param handle
* The accessing object.
* 
* \return cy_en_rtl8211f_speed_t
* Returns the Speed of the Link in RTL8211F \ref cy_en_rtl8211f_speed_t
*
*******************************************************************************/
cy_en_rtl8211f_speed_t Cy_Rtl8211f_GetLinkSpeed(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_physr_t physr;
    cy_un_rtl8211f_pagsr_t pagsr;
    cy_en_rtl8211f_speed_t retVal = ETH_PHY_LINK_SPEED_10M;

    pagsr.f.u12PAGE_SEL = 0xa43;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    physr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYSR);
    
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    switch (physr.f.u2SPEED_SELECTION) 
    {
        case 0x02:
            retVal = ETH_PHY_LINK_SPEED_1000M;
            break;
        case 0x01:
            retVal = ETH_PHY_LINK_SPEED_100M;
            break;
        default:
            retVal = ETH_PHY_LINK_SPEED_10M;
            break;
    }

    return retVal;
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_GetLinkMode
****************************************************************************//**
*
* \brief Function checks and returns the link mode from a RTL8211FIR register
*
* \param handle
* The accessing object.
* 
* \return cy_en_rtl8211f_duplex_t
* Returns the Duplex Mode of the Link in RTL8211F \ref cy_en_rtl8211f_duplex_t
*
*******************************************************************************/
cy_en_rtl8211f_duplex_t Cy_Rtl8211f_GetLinkMode(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_physr_t physr;
    cy_un_rtl8211f_pagsr_t pagsr;

    pagsr.f.u12PAGE_SEL = 0xa43;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    physr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_PHYSR);
    
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    return ((physr.f.u1DUPLEX_MODE) ? ETH_PHY_LINK_DUPLEX_FULL : ETH_PHY_LINK_DUPLEX_HALF);
}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_GetInterruptStatus
****************************************************************************//**
*
* \brief Function to get the Interrupt Status from RTL8211F Register
*
* \param handle
* The accessing object.
* 
* \return uint32_t 
* Return the status value. Check Register for Details.
*
*******************************************************************************/
uint32_t Cy_Rtl8211f_GetInterruptStatus(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_insr_t insr;
    cy_un_rtl8211f_pagsr_t pagsr;

    pagsr.f.u12PAGE_SEL = 0xa43;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    insr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_INSR);
    
    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    return (insr.u16);

}

/*******************************************************************************
* Function Name: Cy_Rtl8211f_ConfigureLeds
****************************************************************************//**
*
* \brief Function to configure the LEDs
*
* \param handle
* The accessing object.
* 
*
*******************************************************************************/
void Cy_Rtl8211f_ConfigureLeds(cy_stc_rtl8211f_t* handle)
{
    cy_un_rtl8211f_lcr_t lcr;
    cy_un_rtl8211f_pagsr_t pagsr;

    pagsr.f.u12PAGE_SEL = 0xd04;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);

    lcr.u16 = Cy_Rtl8211f_ReadReg(handle, CY_RTL8211F_REG_NORMAL_LCR);
    lcr.f.u1LED0_LINK_10 = 1u;
    lcr.f.u1LED0_LINK_1000 = 1u;
    lcr.f.u1LED0_ACT = 1u;
    lcr.f.u1LED1_LINK_100 = 1u;
    lcr.f.u1LED1_LINK_1000 = 1u;
    lcr.f.u1LED1_ACT = 1u;
    lcr.f.u1LED2_LINK_10 = 1u;
    lcr.f.u1LED2_LINK_100 = 1u;
    lcr.f.u1LED2_ACT = 1u;

    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_LCR, lcr.u16);

    pagsr.f.u12PAGE_SEL = 0x0;
    Cy_Rtl8211f_WriteReg(handle, CY_RTL8211F_REG_NORMAL_PAGSR, pagsr.u16);
}

/* [] END OF FILE */

#endif  /* CY_327BGA_EVKLITE_rev_a */
