/***************************************************************************//**
* \file cy_rtl8211f.h
*
* \brief Driver for Ethernet PHY RTL8211f
*
********************************************************************************
* \copyright
* Copyright 2016-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/* Only applicable for the lite kit */
#ifdef CY_327BGA_EVKLITE_rev_a

#ifndef __CY_RTL8211F_H__
#define __CY_RTL8211F_H__


/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"


/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#define RTL8211F_PHY_ID                         0x001cc910      /* PHY_ID for RTL8211F */
#define RTL8211F_PHY_ID_MSK                     0xFFFFFFF0      /* PHY_ID for RTL8211F */

/* MDIO Manageable Devices (MMDs). */
#define MDIO_MMD_PMAPMD		                1	        /* Physical Medium Attachment / Physical Medium Dependent */
#define MDIO_MMD_WIS		                2	        /* WAN Interface Sublayer */
#define MDIO_MMD_PCS		                3	        /* Physical Coding Sublayer */
#define MDIO_MMD_PHYXS		                4	        /* PHY Extender Sublayer */
#define MDIO_MMD_DTEXS		                5	        /* DTE Extender Sublayer */
#define MDIO_MMD_TC		                6	        /* Transmission Convergence */
#define MDIO_MMD_AN		                7	        /* Auto-Negotiation */
#define MDIO_MMD_C22EXT		                29	        /* Clause 22 extension */
#define MDIO_MMD_VEND1		                30	        /* Vendor specific 1 */
#define MDIO_MMD_VEND2		                31	        /* Vendor specific 2 */

#define MDIO_PCS_EEE_ABLE	                20	        /* EEE Capability register */
#define MDIO_PCS_EEE_ABLE2	                21	        /* EEE Capability register 2 */

#define MDIO_AN_EEE_ADV		                60	        /* EEE advertisement */
#define MDIO_AN_EEE_LPABLE	                61	        /* EEE link partner ability */
#define MDIO_AN_EEE_ADV2	                62	        /* EEE advertisement 2 */
#define MDIO_AN_EEE_LPABLE2	                63	        /* EEE link partner ability 2 */

/* MMD Access Control register fields */
#define MII_MMD_CTRL_DEVAD_MASK                 0x1f            /* Mask MMD DEVAD*/
#define MII_MMD_CTRL_ADDR                       0x0000          /* Address */
#define MII_MMD_CTRL_NOINCR                     0x4000          /* no post increment */
#define MII_MMD_CTRL_INCR_RDWT                  0x8000          /* post increment on reads & writes */
#define MII_MMD_CTRL_INCR_ON_WT                 0xC000          /* post increment on writes only */

/* Advertisement control register. */
#define ADVERTISE_SLCT                          0x001f          /* Selector bits               */
#define ADVERTISE_CSMA                          0x0001          /* Only selector supported     */
#define ADVERTISE_10HALF                        0x0020          /* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL                     0x0020          /* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL                        0x0040          /* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF                     0x0040          /* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF                       0x0080          /* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE                    0x0080          /* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL                       0x0100          /* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM                 0x0100          /* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4                      0x0200          /* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP                     0x0400          /* Try for pause               */
#define ADVERTISE_PAUSE_ASYM                    0x0800          /* Try for asymetric pause     */
#define ADVERTISE_RESV                          0x1000          /* Unused...                   */
#define ADVERTISE_RFAULT                        0x2000          /* Say we can detect faults    */
#define ADVERTISE_LPACK                         0x4000          /* Ack link partners response  */
#define ADVERTISE_NPAGE                         0x8000          /* Next page bit               */

#define ADVERTISE_FULL                          (ADVERTISE_100FULL | ADVERTISE_10FULL | ADVERTISE_CSMA)
#define ADVERTISE_ALL                           (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL)

#define ETH_PHY_CONFIG_INTERFACE_Msk            0x7
#define ETH_PHY_CONFIG_INTERFACE_Pos            12
#define ETH_PHY_CONFIG_LINK_MODE_Msk            0x3F
#define ETH_PHY_CONFIG_LINK_MODE_Pos            0
#define ETH_PHY_CONFIG_ADVERTISING_Msk          (1 << 7)
#define ETH_PHY_CONFIG_ADVERTISE_10_100_Msk     0x0f
#define ETH_PHY_CONFIG_ADVERTISE_10_100_Pos     0
#define ETH_PHY_CONFIG_ADVERTISE_1000_Msk       0x30
#define ETH_PHY_CONFIG_ADVERTISE_1000_Pos       4
#define ETH_PHY_MDIO_TIMEOUT                    (0xffffffffUL)
#define ETH_PHY_MDIO_C45                        (0x80000000UL)

#define ETH_PHY_TX_DELAY_EN                     0x0100
#define ETH_PHY_RX_DELAY_EN                     0x0004


/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/


/**
 *****************************************************************************
 ** \brief Addresses of normal PHY registers (addresses 0-31)
 ** 
 *****************************************************************************/

/* BMCR */
typedef struct
{
    uint16_t u5RESERVED                  : 5; /* RESERVED: Writes ignored, read as 0. */
    uint16_t u1UNI_DIRECTIONAL_ENABLE    : 1; /*Uni-Directional Enable 
                                                1: Enable packet transmit without respect to linkok status 
                                                0: Packet transmit permitted when link is established */
    uint16_t u1SPEED_SELECTION_MSB       : 1; /* Speed Select: See description for bit 13. */
    uint16_t u1COLLISION_TEST            : 1; /* Collision Test:
                                                 1 = Collision test enabled.
                                                 0 = Normal operation.
                                                 When set, this bit will cause the COL signal to be asserted in
                                                 response to the assertion of TX_EN within 512-bit times. The COL
                                                 signal will be deasserted within 4-bit times in response to the
                                                 deassertion of TX_EN. */
    uint16_t u1DUPLEX_MODE               : 1; /* Duplex Mode:
                                                 When auto-negotiation is disabled writing to this bit allows the port
                                                 Duplex capability to be selected.
                                                 1 = Full Duplex operation.
                                                 0 = Half Duplex operation. */
    uint16_t u1RESTART_AUTO_NEGOTIATION  : 1; /* Restart Auto-Negotiation:
                                                 1 = Restart Auto-Negotiation. Reinitiates the Auto-Negotiation
                                                 process. If Auto-Negotiation is disabled (bit 12 = 0), this bit is
                                                 ignored. This bit is self-clearing and will return a value of 1 until
                                                 Auto-Negotiation is initiated, whereupon it will self-clear. Operation of
                                                 the Auto-Negotiation process is not affected by the management
                                                 entity clearing this bit.
                                                 0 = Normal operation. */
    uint16_t u1ISOLATE                   : 1; /* Isolate:
                                                 1 = Isolates the Port from the MII with the exception of the serial
                                                 management.
                                                 0 = Normal operation. */
    uint16_t u1POWER_DOWN                : 1; /* Power Down:
                                                 1 = Power down.
                                                 0 = Normal operation.
                                                 Setting this bit powers down the PHY. Only the register block is
                                                 enabled during a power down condition. This bit is ORd with the
                                                 input from the PWRDOWN_INT pin. When the active low
                                                 PWRDOWN_INT pin is asserted, this bit will be set. */
    uint16_t u1AUTO_NEGOTIATION_ENABLE   : 1; /* Auto-Negotiation Enable:
                                                 Strap controls initial value at reset.
                                                 1 = Auto-Negotiation Enabled - bits 8 and 13 of this register are
                                                 ignored when this bit is set.
                                                 0 = Auto-Negotiation Disabled - bits 8 and 13 determine the port
                                                 speed and duplex mode. */
    uint16_t u1SPEED_SELECTION_LSB       : 1; /* Speed Select (Bits 6, 13):
                                                 When auto-negotiation is disabled writing to this bit allows the port
                                                 speed to be selected.
                                                 11 = Reserved
                                                 10 = 1000 Mbps
                                                 1 = 100 Mbps
                                                 0 = 10 Mbps */
    uint16_t u1LOOPBACK                  : 1; /* Loopback:
                                                 1 = Loopback enabled.
                                                 0 = Normal operation.
                                                 The loopback function enables MII transmit data to be routed to the
                                                 MII receive data path.
                                                 Setting this bit may cause the descrambler to lose synchronization
                                                 and produce a 500-s dead time before any valid data will appear at
                                                 the MII receive outputs. */
    uint16_t u1RESET                     : 1; /* Reset:
                                                 1 = Initiate software Reset / Reset in Process.
                                                 0 = Normal operation.
                                                 This bit, which is self-clearing, returns a value of one until the reset
                                                 process is complete. The configuration is restrapped. */
} cy_stc_rtl8211f_bmcr_t;


typedef union
{
    cy_stc_rtl8211f_bmcr_t f;
    uint16_t u16;
} cy_un_rtl8211f_bmcr_t;


/* BMSR */
typedef struct
{
    uint16_t u1EXTENDED_CAPABILITY        : 1; /* Extended Capability:
                                                  1 = Extended register capabilities.
                                                  0 = Basic register set capabilities only. */
    uint16_t u1JABBER_DETECT              : 1; /* Jabber Detect: This bit only has meaning in 10-Mbps mode.
                                                  1 = Jabber condition detected.
                                                  0 = No Jabber.
                                                  This bit is implemented with a latching function, such that the
                                                  occurrence of a jabber condition causes it to set until it is cleared by
                                                  a read to this register by the management interface or by a reset. */
    uint16_t u1LINK_STATUS                : 1; /* Link Status:
                                                  1 = Valid link established.
                                                  0 = Link not established.
                                                  This register indicates whether the link was lost since the last read. 
                                                  For the current link status, either read this register twice or read 
                                                  Page 0xa43 Reg 26, bit[2] Link (Real Time) */
    uint16_t u1AUTO_NEGOTIATION_ABILITY   : 1; /* Auto Negotiation Ability:
                                                  1 = Device is able to perform Auto-Negotiation.
                                                  0 = Device is not able to perform Auto-Negotiation. */
    uint16_t u1REMOTE_FAULT               : 1; /* Remote Fault:
                                                  1 = Remote Fault condition detected (cleared on read or by reset).
                                                  Fault criteria: Far-End Fault Indication or notification from Link
                                                  Partner of Remote Fault.
                                                  0 = No remote fault condition detected. */
    uint16_t u1AUTO_NEGOTIATION_COMPLETE  : 1; /* Auto-Negotiation Complete:
                                                  1 = Auto-Negotiation process complete.
                                                  0 = Auto-Negotiation process not complete. */
    uint16_t u1MF_PREAMBLE_SUPPRESSION    : 1; /* Preamble Suppression Capable:
                                                  1 = Device able to perform management transaction with preamble
                                                  suppressed, 32-bits of preamble needed only once after reset,
                                                  invalid opcode or invalid turnaround.
                                                  0 = Normal management operation. */
    uint16_t u1UNI_DIRECTIONAL_ABILITY    : 1; /* Uni-directional ability. 
                                                  1: PHY able to transmit from RGMII without linkok 
                                                  0: PHY not able to transmit from RGMII without linkok */
    uint16_t u1EXTENDED_STATUS            : 1; /* 1000BASE-T Extended Status Register:
                                                  1 = Device supports Extended Status Register 0x0F. */
    uint16_t u1_10BASE_T2_HALF_DUPLEX     : 1;  /*  100BASE-T2 Half Duplex Capable:
                                                  0 = Device not able to perform 100BASE-T2 in half duplex mode. */
    uint16_t u1_10BASE_T2_FULL_DUPLEX     : 1;  /*  100BASE-T2 Full Duplex Capable:
                                                  0 = Device not able to perform 100BASE-T2 in full duplex mode. */
    uint16_t u1_10BASE_T_HALF_DUPLEX      : 1; /* 10BASE-T Half Duplex Capable:
                                                  1: Device is able to perform 10Base-T in half duplex mode 
                                                  0: Device is not able to perform 10Base-T in half duplex mode */
    uint16_t u1_10BASE_T_FULL_DUPLEX      : 1; /* 10BASE-T Full Duplex Capable:
                                                  1: Device is able to perform 10Base-T in full duplex mode 
                                                  0: Device is not able to perform 10Base-T in full duplex mode*/
    uint16_t u1_100BASE_TX_HALF_DUPLEX    : 1; /* 100BASE-TX Half Duplex Capable:
                                                  1: Device is able to perform 100Base-TX in half duplex mode 
                                                  0: Device is not able to perform 100Base-TX in half duplex mode */
    uint16_t u1_100BASE_TX_FULL_DUPLEX    : 1; /* 100BASE-TX Full Duplex Capable:
                                                  1: Device is able to perform 100Base-TX in full duplex mode 
                                                  0: Device is not able to perform 100Base-TX in full duplex mode  */
    uint16_t u1_100BASE_T4                : 1; /* 100BASE-T4 Capable:
                                                  0 = Device not able to perform 100BASE-T4 mode. */
} cy_stc_rtl8211f_bmsr_t;


typedef union
{
    cy_stc_rtl8211f_bmsr_t f;
    uint16_t u16;
} cy_un_rtl8211f_bmsr_t;


/* PHYIDR1 */
typedef struct
{
    uint16_t u16OUI_MSB  : 16; /* OUI Most Significant Bits: Bits 3 to 18 of the OUI (080028h,) are
                                  stored in bits 15 to 0 of this register. The most significant two bits of
                                  the OUI are ignored (the IEEE standard refers to these as bits 1 and 2). */
} cy_stc_rtl8211f_phyidr1_t;


typedef union
{
    cy_stc_rtl8211f_phyidr1_t f;
    uint16_t u16;
} cy_un_rtl8211f_phyidr1_t;


/* PHYIDR2 */
typedef struct
{
    uint16_t u4MDL_REV   : 4; /* Model Revision Number:
                                 Four bits of the vendor model revision number are mapped from bits
                                 3 to 0 (most significant bit to bit 3). This field will be incremented for
                                 all major device changes. */
    uint16_t u6MANU_MDL  : 6; /* Manufacturer Model Number:
                                 The six bits of vendor model number are mapped from bits 9 to 4
                                 (most significant bit to bit 9). */
    uint16_t u6OUI_LSB   : 6; /* OUI Least Significant Bits:
                                 Bits 19 to 24 of the OUI (080028h) are mapped from bits 15 to 10 of this register respectively. */
} cy_stc_rtl8211f_phyidr2_t;


typedef union
{
    cy_stc_rtl8211f_phyidr2_t f;
    uint16_t u16;
} cy_un_rtl8211f_phyidr2_t;


/* ANAR */
typedef struct
{
    uint16_t u5SELECTOR      : 5; /* Protocol Selection Bits:
                                     These bits contain the binary encoded protocol selector supported by this port.
                                     <00001> indicates that this device supports IEEE 802.3u. */
    uint16_t u1_10_HD        : 1; /* 10BASE-T Support:
                                     1: Advertise support of 10Base-TX half-duplex mode 
                                     0: Not advertised  */
    uint16_t u1_10_FD        : 1; /* 10BASE-Te Full Duplex Support:
                                     1: Advertise support of 10Base-TX full-duplex mode 
                                     0: Not advertised  */
    uint16_t u1TX_HD         : 1; /* 100BASE-TX Support:
                                     1: Advertise support of 100Base-TX half-duplex mode 
                                     0: Not advertised  */
    uint16_t u1TX_FD         : 1; /* 100BASE-TX Full Duplex Support:
                                     1: Advertise support of 100Base-TX full-duplex mode 
                                     0: Not advertised . */
    uint16_t u1T4            : 1; /* 100BASE-T4 Support:
                                     1= 100BASE-T4 is supported by the local device.
                                     0 = 100BASE-T4 not supported. */
    uint16_t u1PAUSE         : 1; /* PAUSE Support for Full Duplex Links:
                                     The PAUSE bit indicates that the device is capable of providing the symmetric PAUSE
                                     functions as defined in Annex 31B.
                                     Encoding and resolution of PAUSE bits is defined in IEEE 802.3 Annex 28B, Tables
                                     28B-2 and 28B-3, respectively. Pause resolution status is reported in PHYCR[13:12].
                                     1 = Advertise that the DTE (MAC) has implemented both the optional MAC control
                                     sublayer and the pause function as specified in clause 31 and annex 31B of 802.3u.
                                     0= No MAC based full duplex flow control. */
    uint16_t u1ASM_DIR       : 1; /* Asymmetric PAUSE Support for Full Duplex Links:
                                     The ASM_DIR bit indicates that asymmetric PAUSE is supported.
                                     Encoding and resolution of PAUSE bits is defined in IEEE 802.3 Annex 28B, Tables
                                     28B-2 and 28B-3, respectively. Pause resolution status is reported in PHYCR[13:12].
                                     1 = Advertise that the DTE (MAC) has implemented both the optional MAC control
                                     sublayer and the pause function as specified in clause 31 and annex 31B of 802.3u.
                                     0= No MAC based full duplex flow control. */
    uint16_t u1RESERVED1     : 1; /* RESERVED for Future IEEE use: Write as 0, Read as 0 */
    uint16_t u1RF            : 1; /* Remote Fault:
                                     1 = Advertises that this device has detected a Remote Fault.
                                     0 = No Remote Fault detected. */
    uint16_t u1RESERVED0     : 1; /* RESERVED by IEEE: Writes ignored, read as 0. */
    uint16_t u1NP            : 1; /* Next Page Indication:
                                     0 = Next Page Transfer not desired.
                                     1 = Next Page Transfer desired. */
} cy_stc_rtl8211f_anar_t;


typedef union
{
    cy_stc_rtl8211f_anar_t f;
    uint16_t u16;
} cy_un_rtl8211f_anar_t;


/* ANLPAR */
typedef struct
{
    uint16_t u5SELECTOR             : 5; /* Protocol Selection Bits:
                                            Link Partners binary encoded protocol selector. */
    uint16_t u7TECH_ABILITY_FIELD   : 7; /*Received Code word bit */
    uint16_t u1RESERVED0            : 1; /* RESERVED for Future IEEE use:Write as 0, read as 0. */
    uint16_t u1RF                   : 1; /* Remote Fault:
                                            1 = Remote Fault indicated by Link Partner.
                                            0 = No Remote Fault indicated by Link Partner. */
    uint16_t u1ACK                  : 1; /* Acknowledge:
                                            1 = Link Partner acknowledges reception of the ability data word.
                                            0 = Not acknowledged.
                                            The Auto-Negotiation state machine will automatically control the this bit based on the
                                            incoming FLP bursts. */
    uint16_t u1NP                   : 1; /* Next Page Indication:
                                            0 = Link Partner does not desire Next Page Transfer.
                                            1 = Link Partner desires Next Page Transfer. */
} cy_stc_rtl8211f_anlpar_t;


typedef union
{
    cy_stc_rtl8211f_anlpar_t f;
    uint16_t u16;
} cy_un_rtl8211f_anlpar_t;


/* ANER */
typedef struct
{
    uint16_t u1LP_AN_ABLE             : 1; /* Link Partner Auto-Negotiation Able:
                                              1 = indicates that the Link Partner supports Auto-Negotiation.
                                              0 = indicates that the Link Partner does not support Auto-Negotiation. */
    uint16_t u1PAGE_RX                : 1; /* Link Code Word Page Received:
                                              1 = Link Code Word has been received, cleared on a read.
                                              0 = Link Code Word has not been received. */
    uint16_t u1NP_ABLE                : 1; /* Next Page Able:
                                              1 = Indicates local device is able to send additional Next Pages. */
    uint16_t u1LP_NP_ABLE             : 1; /* Link Partner Next Page Able:
                                              1 = Link Partner does support Next Page.
                                              0 = Link Partner does not support Next Page. */
    uint16_t u1PDF                    : 1; /* Parallel Detection Fault:
                                              1 = A fault has been detected via the Parallel Detection function.
                                              0 = A fault has not been detected. */
    uint16_t u1RX_NEXT_PAGE_STOR_LOC  : 1; /* Receive Next Page Storage Location:
                                              1 = Link Partner Next Pages are stored in register 8.
                                              0 = Link Partner Next Pages are stored in register 5. */
    uint16_t u1RX_NEXT_PAGE_LOC_ABLE  : 1; /* Receive Next Page Location Able:
                                              1 = Received Next Page storage location is specified by bit 6.5.
                                              0 = Received Next Page storage location is not specified by bit 6.5. */
    uint16_t u9RESERVED               : 9; /* RESERVED: Writes ignored, read as 0. */
} cy_stc_rtl8211f_aner_t;


typedef union
{
    cy_stc_rtl8211f_aner_t f;
    uint16_t u16;
} cy_un_rtl8211f_aner_t;


/* ANNPTR */
typedef struct
{
    uint16_t u11CODE        : 11; /* Code:
                                    This field represents the code field of the next page transmission. If the MP bit is
                                    set (bit 13 of this register), then the code shall be interpreted as a "Message
                                    Page”, as defined in annex 28C of IEEE 802.3u. Otherwise, the code shall be
                                    interpreted as an "Unformatted Page”, and the interpretation is application
                                    specific.
                                    The default value of the CODE represents a Null Page as defined in Annex 28C
                                    of IEEE 802.3u. */
    uint16_t u1TOG_TX       : 1; /* Toggle:
                                    1 = Value of toggle bit in previously transmitted Link Code Word was 0.
                                    0 = Value of toggle bit in previously transmitted Link Code Word was 1.
                                    Toggle is used by the Arbitration function within Auto-Negotiation to ensure
                                    synchronization with the Link Partner during Next Page exchange. This bit shall
                                    always take the opposite value of the Toggle bit in the previously exchanged Link
                                    Code Word. */
    uint16_t u1ACK2         : 1; /* Acknowledge2:
                                    1 = Will comply with message.
                                    0 = Cannot comply with message.
                                    Acknowledge2 is used by the next page function to indicate that Local Device has
                                    the ability to comply with the message received. */
    uint16_t u1MP           : 1; /* Message Page:
                                    1 = Message Page.
                                    0 = Unformatted Page. */
    uint16_t u1RESERVED1    : 1; /* Reserved Bit*/
    uint16_t u1NP           : 1; /* Next Page Indication:
                                    0 = No other Next Page Transfer desired.
                                    1 = Another Next Page desired. */
} cy_stc_rtl8211f_annptr_t;


typedef union
{
    cy_stc_rtl8211f_annptr_t f;
    uint16_t u16;
} cy_un_rtl8211f_annptr_t;


/* ANNPRR */
typedef struct
{
    uint16_t u11MSG    : 11; /* Message */
    uint16_t u1TOG_TX  : 1; /* Toggle:
                               1 = Value of toggle bit in previously transmitted Link Code Word
                               was 0.
                               0 = Value of toggle bit in previously transmitted Link Code Word
                               was 1.
                               Toggle is used by the Arbitration function within Auto-Negotiation to
                               ensure synchronization with the Link Partner during Next Page
                               exchange. This bit shall always take the opposite value of the
                               Toggle bit in the previously exchanged Link Code Word. */
    uint16_t u1ACK2    : 1; /* Acknowledge2:
                               1 = Link partner sets the ACK2 bit.
                               0 = Link partner coes not set the ACK2 bit.
                               Acknowledge2 is used by the next page function to indicate that link
                               partner has the ability to comply with the message received. */
    uint16_t u1MP      : 1; /* Message Page:
                               1 = Received page is a Message Page.
                               0 = Received page is an Unformatted Page. */
    uint16_t u1ACK     : 1; /* Acknowledge:
                               1 = Acknowledge reception of link code word by the link partner.
                               0 = Link partner does not acknowledge reception of link code word. */
    uint16_t u1NP      : 1; /* Next Page Indication:
                               0 = No other Next Page Transfer desired by the link partner.
                               1 = Another Next Page desired by the link partner. */
} cy_stc_rtl8211f_annprr_t;


typedef union
{
    cy_stc_rtl8211f_annprr_t f;
    uint16_t u16;
} cy_un_rtl8211f_annprr_t;


/* GBCR */
typedef struct
{
    uint16_t u8RESERVED                           : 7; /* RESERVED: Write ignored, read as 0. */
    uint16_t u1RESERVED1                          : 1; /* Reserved: Write ignored, read as 0. */
    uint16_t u1_1000BASE_T_FULL_DUPLEX            : 1; /* Advertise 1000BASE-T Full Duplex Capable:
                                                          1 = Advertise 1000Base-T Full Duplex ability.
                                                          0 = Do not advertise 1000Base-T Full Duplex ability. */
    uint16_t u1PORT_TYPE                          : 1; /* Advertise Device Type: Multi or single port:
                                                          1 = Multi-port device.
                                                          0 = Single-port device. */
    uint16_t u1MASTER_SLAVE_CONFIGURATION_VALUE   : 1; /* Manual Master / Slave Configuration Value:
                                                          1 = Set PHY as MASTER when register 09h bit 12 = 1.
                                                          0 = Set PHY as SLAVE when register 09h bit 12 = 1.
                                                          Using the manual configuration feature may prevent the PHY from
                                                          establishing link in 1000Base-T mode if a conflict with the link
                                                          partner’s setting exists. */
    uint16_t u1MASTER_SLAVE_MANUAL_CONFIGURATION  : 1; /* Enable Manual Master / Slave Configuration:
                                                          1 = Enable Manual Master/Slave Configuration control.
                                                          0 = Disable Manual Master/Slave Configuration control.
                                                          Using the manual configuration feature may prevent the PHY from
                                                          establishing link in 1000Base-T mode if a conflict with the link
                                                          partner’s setting exists. */
    uint16_t u3TEST_MODE                          : 3; /* Test Mode Select:
                                                          111,110,101 : Reserved
                                                          100 = Test Mode 4 - Transmit Distortion Test
                                                          011 = Test Mode 3 - Transmit Jitter Test (Slave Mode)
                                                          010 = Test Mode 2 - Transmit Jitter Test (Master Mode)
                                                          001 = Test Mode 1 - Transmit Jitter Test
                                                          000 = Normal Mode */
} cy_stc_rtl8211f_gbcr_t; //cy_stc_rtl8211f_cfg1_t


typedef union
{
    cy_stc_rtl8211f_gbcr_t f;
    uint16_t u16;
} cy_un_rtl8211f_gbcr_t; //cy_un_rtl8211f_cfg1_t


/* GBSR*/
typedef struct
{
    uint16_t u8IDLE_ERROR_COUNTER                     : 8; /* 1000BASE-T Idle Error Counter */
    uint16_t u2RESERVED                               : 2; /* RESERVED by IEEE: Writes ignored, read as 0. */
    uint16_t u1_1000BASE_T_HALF_DUPLEX                : 1; /* Link Partner 1000BASE-T Half Duplex Capable:
                                                              1 = Link Partner capable of 1000Base-T Half Duplex.
                                                              0 = Link partner not capable of 1000Base-T Half Duplex. */
    uint16_t u1_1000BASE_T_FULL_DUPLEX                : 1; /* Link Partner 1000BASE-T Full Duplex Capable:
                                                              1 = Link Partner capable of 1000Base-T Full Duplex.
                                                              0 = Link partner not capable of 1000Base-T Full Duplex. */
    uint16_t u1REMOTE_RECEIVER_STATUS                 : 1; /* Remote Receiver Status:
                                                              1 = Remote receiver is OK.
                                                              0 = Remote receiver is not OK. */
    uint16_t u1LOCAL_RECEIVER_STATUS                  : 1; /* Local Receiver Status:
                                                              1 = Local receiver is OK.
                                                              0 = Local receiver is not OK. */
    uint16_t u1MASTER_SLAVE_CONFIGURATION_RESOLUTION  : 1; /* Master / Slave Configuration Results:
                                                              1 = Configuration resolved to MASTER.
                                                              0 = Configuration resolved to SLAVE. */
    uint16_t u1MASTER_SLAVE_CONFIGURATION_FAULT       : 1; /* Master / Slave Manual Configuration Fault Detected:
                                                              1 = Manual Master/Slave Configuration fault detected.
                                                              0 = No Manual Master/Slave Configuration fault detected. */
} cy_stc_rtl8211f_gbsr_t; //cy_stc_rtl8211f_sts1_t


typedef union
{
    cy_stc_rtl8211f_gbsr_t f;
    uint16_t u16;
} cy_un_rtl8211f_gbsr_t; //cy_un_rtl8211f_sts1_t


/* MACR */
typedef struct
{
    uint16_t u5DEVAD     : 5; /* Device Address: In general, these bits [4:0] are the device address
                                 DEVAD that directs any accesses of ADDAR register (0x000E) to
                                 the appropriate MMD. Specifically, the rtl8211f uses the vendor
                                 specific DEVAD [4:0] = 11111 for accesses. All accesses through
                                 registers REGCR and ADDAR should use this DEVAD.
                                 Transactions with other DEVAD are ignored. */
    uint16_t u9RESERVED  : 9; /* RESERVED: Writes ignored, read as 0. */
    uint16_t u2FUNCTION  : 2; /* 00 = Address
                                 01 = Data, no post increment
                                 10 = Data, post increment on read and write
                                 11 = Data, post increment on write only */
} cy_stc_rtl8211f_macr_t; //cy_stc_rtl8211f_regcr_t


typedef union
{
    cy_stc_rtl8211f_macr_t f;
    uint16_t u16;
} cy_un_rtl8211f_macr_t; // cy_stc_rtl8211f_regcr_t


/* MADDR */
typedef struct
{
    uint16_t u16ADDRESS_DATA  : 16; /* If REGCR register 15:14 = 00, holds the MMD DEVAD's address
                                       register, otherwise holds the MMD DEVAD's data register */
} cy_stc_rtl8211f_maddr_t;


typedef union
{
    cy_stc_rtl8211f_maddr_t f;
    uint16_t u16;
} cy_un_rtl8211f_maddr_t;


/* GBESR */
typedef struct
{
    uint16_t u12RESERVED                     : 12; /* RESERVED by IEEE: Writes ignored, read as 0. */
    uint16_t u1_1000BASE_T_HALF_DUPLEX      : 1; /* 1000BASE-T Half Duplex Support:
                                                    1 = 1000BASE-T Half Duplex is supported by the local device.
                                                    0 = 1000BASE-T Half Duplex is not supported by the local device. */
    uint16_t u1_1000BASE_T_FULL_DUPLEX      : 1; /* 1000BASE-T Full Duplex Support:
                                                    1 = 1000BASE-T Full Duplex is supported by the local device.
                                                    0 = 1000BASE-T Full Duplex is not supported by the local device. */
    uint16_t u1_1000BASE_X_HALF_DUPLEX      : 1; /* 1000BASE-X Half Duplex Support:
                                                    1 = 1000BASE-X Half Duplex is supported by the local device.
                                                    0 = 1000BASE-X Half Duplex is not supported by the local device. */
    uint16_t u1_1000BASE_X_FULL_DUPLEX      : 1; /* 1000BASE-X Full Duplex Support:
                                                    1 = 1000BASE-X Full Duplex is supported by the local device.
                                                    0 = 1000BASE-X Full Duplex is not supported by the local device. */
} cy_stc_rtl8211f__gbesr_t; //cy_stc_rtl8211f__1kscr_t


typedef union
{
    cy_stc_rtl8211f__gbesr_t f;
    uint16_t u16;
} cy_un_rtl8211f__gbesr_t; // cy_un_rtl8211f__1kscr_t


/* PHYCR1 */
typedef struct
{
    uint16_t u1RESERVED0           : 1; /* RESERVED */
    uint16_t u1ALDPS_PLL_OFF_EN    : 1; /* 1: Enable PLL off when in ALDPS mode */
    uint16_t u1ALDPS_EN            : 1; /* 1: Enable Link Down Power Saving Mode */
    uint16_t u1JABBER_DET_EN       : 1; /* 1: Enable Jabber Detection */
    uint16_t u1PREAMBL_CHK_EN      : 1; /* 1: Check preamble when receiving an 
                                           MDC/MDIO command */
    uint16_t u1RESERVED1           : 1; /* RESERVED */     
    uint16_t u1PHYAD_NZ_DETECT     : 1; /* 1: The RTL8211F(I)/RTL8211FD(I) with 
                                           PHYAD[2:0] = 000 will latch the first non-zero PHY 
                                           address as its own PHY address */ 
    uint16_t u1TX_CRS_EN           : 1; /* 1: Assert CRS on transmit 
                                           0: Never assert CRS on transmit */   
    uint16_t u1MDI_MODE            : 1; /* Set the MDI/MDIX mode. 
                                           1: MDI 
                                           0: MDIX */
    uint16_t u1MDI_MANU_CFG_EN     : 1; /* 1: Enable Manual Configuration of MDI mode */ 
    uint16_t u2RESERVED0           : 2; /* RESERVED */
    uint16_t u1ALDPS_XTAL_OFF_EN   : 1; /* 1: Enable XTAL off when in ALDPS mode */
    uint16_t PHYAD_0_EN            : 1; /* 1: A broadcast from MAC (A command with PHY 
                                           address = 0) is valid. 
                                           MDC/MDIO will respond to this command. */
    uint16_t u2RESERVED1           : 2; /* RESERVED */

} cy_stc_rtl8211f_phycr1_t;


typedef union
{
    cy_stc_rtl8211f_phycr1_t f;
    uint16_t u16;
} cy_un_rtl8211f_phycr1_t;

/* PHYCR2 */
typedef struct
{   
    uint16_t u1CLKOUT_EN                : 1; /* 1: CLKOUT Clock Output Enabled */
    uint16_t u1RXC_EN                   : 1; /* 1: RXC Clock Output Enabled. */
    uint16_t u1RESERVED0                : 1; /* RESERVED */
    uint16_t u1SYSCLK_SSC_EN            : 1; /* 1: Enable Spread-Spectrum Clocking (SSC) on 
                                                System Clock. */
    uint16_t u1RESERVED1                : 1; /* RESERVED */
    uint16_t u1PHY_EEE_EN               : 1; /* 1: Enable EEE in PHY mode */
    uint16_t u1RESERVED2                : 1; /* RESERVED */
    uint16_t u1CLKOUT_SSC_EN            : 1; /* 1: Enable Spread-Spectrum Clocking (SSC) on 
                                                CLKOUT output clock. */
    uint16_t u3RESERVED0                : 3; /* RESERVED */
    uint16_t u1CLKOUT_FREQ_SEL          : 1; /* Frequency Select of the CLKOUT Pin Clock Output. 
                                                0: 25MHz 
                                                1: 125MHz */
    uint16_t u2CLKOUT_SSC_CAPB          : 2; /* CLKOUT SSC Capability Select. 
                                                00: Normal CLKOUT. 
                                                11: CLKOUT with SSC capability. 
                                                Others: Reserved. */
    uint16_t u2RESERVED0                : 2; /* RESERVED */                                            

} cy_stc_rtl8211f_phycr2_t;


typedef union
{
    cy_stc_rtl8211f_phycr2_t f;
    uint16_t u16;
} cy_un_rtl8211f_phycr2_t;


/* PHYSR */
typedef struct
{
    uint16_t u1JABBER_DETECT          : 1; /* Jabber Detect: This bit only has meaning in 10 Mbps mode.
                                              This bit is a duplicate of the Jabber Detect bit in the BMSR register,
                                              except that it is not cleared upon a read of the PHYSTS register.
                                              1 = Jabber condition detected.
                                              0 = No Jabber. */
    uint16_t u1MDI_X_MODE             : 1; /* MDI/MDIX Resolution Status 
                                              1 = Resolved as MDI
                                              0 = Resolved as MDIX. */
    uint16_t u1LINK_STATUS            : 1; /* Link Status:
                                              1 = Link is up.
                                              0 = Link is down. */ 
    uint16_t u1DUPLEX_MODE            : 1; /* Duplex Mode Status:
                                              1 = Full Duplex
                                              0 = Half Duplex. */  
    uint16_t u2SPEED_SELECTION        : 2; /* Speed Select Status:
                                              These two bits indicate the speed of operation as determined by
                                              Auto-Negotiation or as set by manual configuration.
                                              11 = Reserved
                                              10 = 1000 Mbps
                                              01 = 100 Mbps
                                              00 = 10 Mbps */
    uint16_t u1TX_FLOW_EN             : 1; /* Tx Flow Control. 
                                              1: Enable 
                                              0: Disable */                                           
    uint16_t u1RX_FLOW_EN             : 1; /* Rx Flow Control. 
                                              1: Enable 
                                              0: Disable */
    uint16_t EEE_CAPB                 : 1; /* 1: Both local and link-partner have 
                                              EEE capability of current speed*/
    uint16_t u2RESERVED0              : 2; /* RESERVED */ 
    uint16_t u1MASTER_MODE            : 1; /* Device is in Master/Slave Mode. 
                                              1: Master mode 
                                              0: Slave mode */ 
    uint16_t u1NWAY_EN                : 1; /* Auto-Negotiation (NWay) Status. 
                                              1: Enable 
                                              0: Disable */                                                                                  
    uint16_t u1MDI_PLUG               : 1; /* MDI Status. 
                                              1: Plugged 
                                              0: Unplugged */
    uint16_t u1ALDPS_STATE            : 1; /* Link Down Power Saving Mode. 
                                              1: Reflects local device entered Link Down Power Saving Mode, 
                                              i.e., cable not plugged in (reflected after 3 sec). 
                                              0: With cable plugged in */
    uint16_t u1RESERVED0              : 1; /* RESERVED */                                                                                    

} cy_stc_rtl8211f_physr_t; //cy_stc_rtl8211f_physts_t


typedef union
{
    cy_stc_rtl8211f_physr_t f;
    uint16_t u16;
} cy_un_rtl8211f_physr_t; //cy_un_rtl8211f_physts_t


/* INER */
typedef struct
{
    uint16_t u1AUTONEG_ERR_INT_EN         : 1; /* Enable Auto-Negotiation Error Interrupt:
                                                  1 = Enable Auto-Negotiation Error interrupt.
                                                  0 = Disable Auto-Negotiation Error interrupt. */
    uint16_t u1RESERVED0                  : 1; /* RESERVED */  
    uint16_t u1PAGE_RECEIVED_INT_EN       : 1; /* Enable Page Received Interrupt:
                                                  1 = Enable Page Received Interrupt.
                                                  0 = Disable Page Received Interrupt. */
    uint16_t u1AUTONEG_COMP_INT_EN        : 1; /* Enable Auto-Negotiation Complete Interrupt:
                                                  1 = Enable Auto-Negotiation Complete Interrupt.
                                                  0 = Disable Auto-Negotiation Complete Interrupt. */
    uint16_t u1LINK_STATUS_CHNG_INT_EN    : 1; /* Enable Link Status Change Interrupt:
                                                  1 = Enable Link Status Change interrupt.
                                                  0 = Disable Link Status Change interrupt. */
    uint16_t u1PHY_REG_ACC_INT_EN         : 1; /* 1: Interrupt Enable 
                                                  0: Interrupt Disable  */                                              
    uint16_t u1RESERVED1                  : 1; /* RESERVED */     
    uint16_t u1PME_INT_EN                 : 1; /*Power Management Event of WOL
                                                  1: Interrupt Enable 
                                                  0: Interrupt Disable */ 
    uint16_t u1RESERVED2                  : 1; /* RESERVED */     
    uint16_t u1ALDPS_CHG_INT_EN           : 1; /* ALDPS State Change Interrupt
                                                  1: Interrupt Enable 
                                                  0: Interrupt Disable */ 
    uint16_t u1JABBER_INT_EN              : 1; /* Enable Jabber Interrupt:
                                                  1 = Enable Jabber interrupt.
                                                  0 = Disable Jabber interrupt. */
    uint16_t u5RESERVED3                  : 5; /* RESERVED */
    
} cy_stc_rtl8211f_iner_t; // cy_stc_rtl8211f_micr_t


typedef union
{
    cy_stc_rtl8211f_iner_t f;
    uint16_t u16;
} cy_un_rtl8211f_iner_t; //cy_un_rtl8211f_micr_t


/* INSR */
typedef struct
{
    uint16_t u1AUTONEG_ERR_INT         : 1; /* Enable Auto-Negotiation Error Interrupt:
                                               1 = Auto-Negotiation Error interrupt.
                                               0 = Auto-Negotiation Error interrupt. */
    uint16_t u1RESERVED0               : 1; /* RESERVED */  
    uint16_t u1PAGE_RECEIVED_INT       : 1; /* Enable Page Received Interrupt:
                                               1 = Page Received Interrupt.
                                               0 = Page Received Interrupt. */
    uint16_t u1AUTONEG_COMP_INT        : 1; /* Enable Auto-Negotiation Complete Interrupt:
                                               1 = Enable Auto-Negotiation Complete Interrupt.
                                               0 = Disable Auto-Negotiation Complete Interrupt. */
    uint16_t u1LINK_STATUS_CHNG_INT    : 1; /* Enable Link Status Change Interrupt:
                                               1 = Enable Link Status Change interrupt.
                                               0 = Disable Link Status Change interrupt. */
    uint16_t u1PHY_REG_ACC_INT         : 1; /* 1: Interrupt Enable 
                                               0: Interrupt Disable  */                                              
    uint16_t u1RESERVED1               : 1; /* RESERVED */     
    uint16_t u1PME_INT                 : 1; /*Power Management Event of WOL
                                               1: Interrupt Enable 
                                               0: Interrupt Disable */ 
    uint16_t u1RESERVED2               : 1; /* RESERVED */     
    uint16_t u1ALDPS_CHG_INT           : 1; /* ALDPS State Change Interrupt
                                               1: Interrupt Enable 
                                               0: Interrupt Disable */ 
    uint16_t u1JABBER_INT              : 1; /* Enable Jabber Interrupt:
                                               1 = Enable Jabber interrupt.
                                               0 = Disable Jabber interrupt. */
    uint16_t u5RESERVED0               : 5; /* RESERVED */

} cy_stc_rtl8211f_insr_t;


typedef union
{
    cy_stc_rtl8211f_insr_t f;
    uint16_t u16;
} cy_un_rtl8211f_insr_t;

/* PAGSR */
typedef struct
{
    uint16_t u12PAGE_SEL    : 16; /* Page Select (in HEX). 
                                     0xa42: Page 0xa42 (default page) */
    uint16_t u4RESERVED     : 4; /* RESERVED */                                  
} cy_stc_rtl8211f_pagsr_t;


typedef union
{
    cy_stc_rtl8211f_pagsr_t f; //cy_stc_rtl8211f_recr_t
    uint16_t u16;
} cy_un_rtl8211f_pagsr_t; //cy_un_rtl8211f_recr_t

/* PHYSCR */
typedef struct
{
    uint16_t u1RESERVED         : 1;  /* RESERVED*/
    uint16_t u1PHY_SPL_CFG      : 1;  /* 1: Write 1 to indicate the special PHY 
                                         parameter configuration has been done.*/
    uint16_t u14RESERVED        : 14; /* RESERVED */                                  
} cy_stc_rtl8211f_physcr_t;


typedef union
{
    cy_stc_rtl8211f_physcr_t f; 
    uint16_t u16;
} cy_un_rtl8211f_physcr_t; 


/* LCR */
typedef struct
{
   uint16_t u1LED0_LINK_10      : 1; /* LED0 Link Indication: 10Mbps */
   uint16_t u1LED0_LINK_100     : 1; /* LED0 Link Indication: 100Mbps  */
   uint16_t u1RESERVED0         : 1; /* RESERVED */
   uint16_t u1LED0_LINK_1000    : 1; /* LED0 Link Indication: 1000Mbps */
   uint16_t u1LED0_ACT          : 1; /* LED0 Active (Transmitting or Receiving) Indication */
   uint16_t u1LED1_LINK_10      : 1; /* LED1 Link Indication: 10Mbps */
   uint16_t u1LED1_LINK_100     : 1; /* LED1 Link Indication: 100Mbps  */
   uint16_t u1RESERVED1         : 1; /* RESERVED */
   uint16_t u1LED1_LINK_1000    : 1; /* LED1 Link Indication: 1000Mbps */
   uint16_t u1LED1_ACT          : 1; /* LED1 Active (Transmitting or Receiving) Indication */
   uint16_t u1LED2_LINK_10      : 1; /* LED2 Link Indication: 10Mbps */
   uint16_t u1LED2_LINK_100     : 1; /* LED2 Link Indication: 100Mbps  */
   uint16_t u1RESERVED2         : 1; /* RESERVED */
   uint16_t u1LED2_LINK_1000    : 1; /* LED2 Link Indication: 1000Mbps */
   uint16_t u1LED2_ACT          : 1; /* LED2 Active (Transmitting or Receiving) Indication */
   uint16_t u1LED_MODE          : 1; /* 0: LED Mode A is selected. 
                                        1: LED Mode B is selected. */

} cy_stc_rtl8211f_lcr_t;


typedef union
{
    cy_stc_rtl8211f_lcr_t f; 
    uint16_t u16;
} cy_un_rtl8211f_lcr_t; 


/* EEELCR */
typedef struct
{
    uint16_t u1RESERVED0        : 1;  /* RESERVED */
    uint16_t u1LED0_EEE_EN      : 1;  /* 1: Enable EEE LED indication of LED0 */
    uint16_t u1LED1_EEE_EN      : 1;  /* 1: Enable EEE LED indication of LED1 */
    uint16_t u1LED2_EEE_EN      : 1;  /* 1: Enable EEE LED indication of LED2 */
    uint16_t u12RESERVED0       : 12; /* RESERVED */        
                                
} cy_stc_rtl8211f_eeelcr_t;

typedef union
{
    cy_stc_rtl8211f_eeelcr_t f; 
    uint16_t u16;
} cy_un_rtl8211f_eeelcr_t; 

/* MIICR */
typedef struct
{
    uint16_t u6RESERVED         : 6;  /* RESERVED*/
    uint16_t u1RGMII_CRS_EN     : 1;  /* 1: Enable in-band CRS Status in RGMII Rx flow */
    uint16_t u9RESERVED         : 9;  /* RESERVED */                                  
} cy_stc_rtl8211f_miicr_t;


typedef union
{
    cy_stc_rtl8211f_miicr_t f; 
    uint16_t u16;
} cy_un_rtl8211f_miicr_t; 


/* INTBCR */
typedef struct
{
    uint16_t u5RESERVED         : 5;  /* RESERVED*/
    uint16_t u1INTB_SEL         : 1;  /* Pin 31 of the RTL8211F(D)(I) functions as: 
                                         1: PMEB 
                                         0: INTB */
    uint16_t u10RESERVED        : 10; /* RESERVED */                                  
} cy_stc_rtl8211f_intbcr_t;


typedef union
{
    cy_stc_rtl8211f_intbcr_t f; 
    uint16_t u16;
} cy_un_rtl8211f_intbcr_t; 


/* PHY registers values */
typedef enum
{
    CY_RTL8211F_REG_NORMAL_BMCR               = 0x0000, // Basic Mode Control Register  \ref cy_un_rtl8211f_bmcr_t
    CY_RTL8211F_REG_NORMAL_BMSR               = 0x0001, // Basic Mode Status Register  \ref cy_un_rtl8211f_bmsr_t
    CY_RTL8211F_REG_NORMAL_PHYIDR1            = 0x0002, // PHY Identifier Register #1  \ref cy_un_rtl8211f_phyidr1_t
    CY_RTL8211F_REG_NORMAL_PHYIDR2            = 0x0003, // PHY Identifier Register #2  \ref cy_un_rtl8211f_phyidr2_t
    CY_RTL8211F_REG_NORMAL_ANAR               = 0x0004, // Auto-Negotiation Advertisement Register  \ref cy_un_rtl8211f_anar_t
    CY_RTL8211F_REG_NORMAL_ANLPAR             = 0x0005, // Auto-Negotiation Link Partner Ability Register (Base Page)  \ref cy_un_rtl8211f_anlpar_t
    CY_RTL8211F_REG_NORMAL_ANER               = 0x0006, // Auto-Negotiation Expansion Register  \ref cy_un_rtl8211f_aner_t
    CY_RTL8211F_REG_NORMAL_ANNPTR             = 0x0007, // Auto-Negotiation Next Page TX  \ref cy_un_rtl8211f_annptr_t
    CY_RTL8211F_REG_NORMAL_ANNPRR             = 0x0008, // Auto-Negotiation Next Page Transmit Register  \ref cy_un_rtl8211f_annprr_t
    CY_RTL8211F_REG_NORMAL_GBCR               = 0x0009, // 1000BASE-T Configuration Register  \ref cy_un_rtl8211f_gbcr_t
    CY_RTL8211F_REG_NORMAL_GBSR               = 0x000A, // 1000BASE-T Status Register  \ref cy_un_rtl8211f_gbsr_t
    CY_RTL8211F_REG_NORMAL_MACR               = 0x000D, // Register Control Register  \ref cy_un_rtl8211f_macr_t
    CY_RTL8211F_REG_NORMAL_MADDR              = 0x000E, // Address or Data Register  \ref cy_un_rtl8211f_maddr_t
    CY_RTL8211F_REG_NORMAL_GBESR              = 0x000F, // 1000BASE-T Extended Status Register  \ref cy_un_rtl8211f_gbesr_t
    CY_RTL8211F_REG_NORMAL_LCR                = 0x0010, // LED Control Register \ref cy_un_rtl8211f_lcr_t
    CY_RTL8211F_REG_NORMAL_EEELCR             = 0x0011, // EEE LED Control Register \ref cy_un_rtl8211f_eeelcr_t
    CY_RTL8211F_REG_NORMAL_INER               = 0x0012, // Interrupt Enable Register  \ref cy_un_rtl8211f_iner_t
    CY_RTL8211F_REG_NORMAL_ISR                = 0x0013, // Interrupt Status Register  \ref cy_un_rtl8211f_isr_t
    CY_RTL8211F_REG_NORMAL_PHYSCR             = 0x0014, // PHY Special Cofig Register \ref cy_un_rtl8211f_physcr_t
    CY_RTL8211F_REG_NORMAL_MIICR              = 0x0015, // MII Control Register \ref cy_un_rtl8211f_miicr_t
    CY_RTL8211F_REG_NORMAL_INTBCR             = 0x0016, // INTB Pin Control Register \ref cy_un_rtl8211f_intbcr_t
    CY_RTL8211F_REG_NORMAL_PHYCR1             = 0x0018, // PHY Specific Control Register 1 \ref cy_un_rtl8211f_phycr1_t
    CY_RTL8211F_REG_NORMAL_PHYCR2             = 0x0019, // PHY Specific Control Register 2  \ref cy_un_rtl8211f_phycr2_t
    CY_RTL8211F_REG_NORMAL_PHYSR              = 0x001A, // PHY Specific Status Register \ref cy_un_rtl8211f_physr_t
    CY_RTL8211F_REG_NORMAL_INSR               = 0x001D, // Interrupt Status Register \ref cy_un_rtl8211f_insr_t
    CY_RTL8211F_REG_NORMAL_PAGSR              = 0x001F, // Page Select Register \ref cy_un_rtl8211f_pagsr_t
} cy_en_rtl8211f_reg_info_t;


/* Ethernet PHY driver handle */
typedef struct
{
    volatile stc_ETH_t* pEth;    /**  Pointer to ETH IP base */
    uint8_t             phyAddr; /**  PHY address (0-31) */
} cy_stc_rtl8211f_t;


/* Ethernet media independent interface mode */
typedef enum 
{
    ETH_PHY_CONFIG_INTERFACE_MII        = 0u,
    ETH_PHY_CONFIG_INTERFACE_RMII       = 1u,
    ETH_PHY_CONFIG_INTERFACE_GMII       = 2u,
    ETH_PHY_CONFIG_INTERFACE_RGMII      = 3u,
    ETH_PHY_CONFIG_INTERFACE_RGMII_ID   = 4u,
    ETH_PHY_CONFIG_INTERFACE_RGMII_RXID = 5u,
    ETH_PHY_CONFIG_INTERFACE_RGMII_TXID = 6u
} cy_en_rtl8211f_interface_mode_t;

/* Ethernet speed mode values */
typedef enum 
{
    ETH_PHY_CONFIG_LINK_MODE_10BASET_HALF       = 0u,
    ETH_PHY_CONFIG_LINK_MODE_10BASET_FULL       = 1u, 
    ETH_PHY_CONFIG_LINK_MODE_100BASET_HALF      = 2u,
    ETH_PHY_CONFIG_LINK_MODE_100BASET_FULL      = 3u,
    ETH_PHY_CONFIG_LINK_MODE_1000BASET_HALF     = 4u,
    ETH_PHY_CONFIG_LINK_MODE_1000BASET_FULL     = 5u
} cy_en_rtl8211f_speed_mode_t;

/* Ethernet advertising mode values */
typedef enum 
{
    ETH_PHY_CONFIG_ADVERTISING_SUPPORTED_10BASET_HALF     = 0u,
    ETH_PHY_CONFIG_ADVERTISING_SUPPORTED_10BASET_FULL     = 1u,
    ETH_PHY_CONFIG_ADVERTISING_SUPPORTED_100BASET_HALF    = 2u,
    ETH_PHY_CONFIG_ADVERTISING_SUPPORTED_100BASET_FULL    = 3u,
    ETH_PHY_CONFIG_ADVERTISING_SUPPORTED_1000BASET_HALF   = 4u,
    ETH_PHY_CONFIG_ADVERTISING_SUPPORTED_1000BASET_FULL   = 5u
} cy_en_rtl8211f_adv_mode_t;

/* Ethernet clock out mode */
typedef enum 
{
    ETH_PHY_CONFIG_ENABLE_CLKOUT_25MHZ  = 0u,
    ETH_PHY_CONFIG_ENABLE_CLKOUT_125MHZ = 1u
} cy_en_rtl8211f_clk_out_t;

/* \brief Ethernet PHY status values */
typedef enum 
{
    ETH_PHY_STATUS_OK             = 0u,            /**< OK. All is fine! */
    ETH_PHY_STATUS_BUSY           = 1u,            /**< Busy */
    ETH_PHY_STATUS_ERROR          = 2u,            /**< Error */
    ETH_PHY_STATUS_ERROR_ID       = 3u,            /**< Error in device identifier */
    ETH_PHY_STATUS_ERROR_TIMEOUT  = 4u,            /**< Time-out error */  
    ETH_PHY_STATUS_ERROR_IF       = 5u,            /**< Interface mode not supported */
    ETH_PHY_STATUS_ERROR_PHYANEG  = 6u,
    ETH_PHY_STATUS_ERROR_SPEED    = 7u,
    ETH_PHY_STATUS_ERROR_PHYREMRX = 8u
} cy_en_rtl8211f_status_t;

/* Ethernet PHY link speed values */
typedef enum 
{
    ETH_PHY_LINK_SPEED_10M      = 0u,
    ETH_PHY_LINK_SPEED_100M     = 1u,
    ETH_PHY_LINK_SPEED_1000M    = 2u,
    ETH_PHY_LINK_SPEED_2500M    = 3u,
    ETH_PHY_LINK_SPEED_5000M    = 4u,
    ETH_PHY_LINK_SPEED_UNKOWN   = 5u
} cy_en_rtl8211f_speed_t;

/* Ethernet PHY link mode values */
typedef enum 
{
    ETH_PHY_LINK_DUPLEX_HALF    = 0u,
    ETH_PHY_LINK_DUPLEX_FULL    = 1u,
    ETH_PHY_LINK_DUPLEX_UNKOWN  = 2u
} cy_en_rtl8211f_duplex_t;


/* Ethernet PHY configuration structure */
typedef struct 
{
    cy_en_rtl8211f_interface_mode_t ifMode;         /**  Interface mode between MAC and PHY */
    cy_en_rtl8211f_speed_mode_t     speedMode;      /**  Speed mode that shall be advertised (#enableAutoNeg == true) or configured */
    cy_en_rtl8211f_adv_mode_t       advMode;        /**  Advertise the speed Supported */
    cy_en_rtl8211f_clk_out_t        clkout;         /**  Clock Out Selection */
    bool                            enableAutoNeg;  /**  Selects between auto-negotation and fixed configuration */
    bool                            enableLoopback; /**  Selects if LoopBack is enabled or not */
    bool                            masterMode;     /** Selects if master Mode is enabled */
} cy_stc_rtl8211f_phy_cfg_t;


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern uint32_t                 Cy_Rtl8211f_Init(cy_stc_rtl8211f_t* handle ,cy_stc_rtl8211f_phy_cfg_t * phyCfg);
extern uint16_t                 Cy_Rtl8211f_ReadReg(cy_stc_rtl8211f_t* handle, cy_en_rtl8211f_reg_info_t reg);
extern void                     Cy_Rtl8211f_WriteReg(cy_stc_rtl8211f_t* handle, cy_en_rtl8211f_reg_info_t reg, uint16_t data);
extern bool                     Cy_Rtl8211f_GetLinkStatus(cy_stc_rtl8211f_t* handle);
extern cy_en_rtl8211f_speed_t   Cy_Rtl8211f_GetLinkSpeed(cy_stc_rtl8211f_t* handle);
extern cy_en_rtl8211f_duplex_t  Cy_Rtl8211f_GetLinkMode(cy_stc_rtl8211f_t* handle);
extern uint32_t                 Cy_Rtl8211f_GetInterruptStatus(cy_stc_rtl8211f_t* handle);
extern void                     Cy_Rtl8211f_ConfigureLeds(cy_stc_rtl8211f_t* handle);
extern void                     Cy_Rtl8211f_Reset(cy_stc_rtl8211f_t* handle);
extern bool                     Cy_Rtl8211f_IsResetDone(cy_stc_rtl8211f_t* handle);
extern void                     Cy_Rtl8211f_Suspend(cy_stc_rtl8211f_t* handle);
extern void                     Cy_Rtl8211f_Resume(cy_stc_rtl8211f_t* handle);
extern bool                     Cy_Rtl8211f_GetPhyID(cy_stc_rtl8211f_t* handle, uint32_t phy_id, uint32_t phy_id_msk);
extern bool                     Cy_Rtl8211f_IsAutonegotiationEnabled(cy_stc_rtl8211f_t* handle);
extern bool                     Cy_Rtl8211f_IsAutonegotiationDone(cy_stc_rtl8211f_t* handle);
extern bool                     Cy_Rtl8211f_IsRemoteReceiverOk(cy_stc_rtl8211f_t* handle);
extern void                     Cy_Rtl8211f_WriteMmdIndirect(cy_stc_rtl8211f_t* handle, uint8_t dev_addr, uint16_t mmd_addr, uint16_t data);
extern uint16_t                 Cy_Rtl8211f_ReadMmdIndirect(cy_stc_rtl8211f_t* handle, uint8_t dev_addr, uint16_t mmd_addr, uint16_t data);
extern void                     Cy_Rtl8211f_ClearEeeAdv(cy_stc_rtl8211f_t* handle);
extern void                     Cy_Rtl8211f_HandleInit(volatile stc_ETH_t* pEth, uint8_t phyAddr, cy_stc_rtl8211f_t* handle);


#endif /* __CY_RTL8211F_H__ */

#endif  /* CY_327BGA_EVKLITE_rev_a */
