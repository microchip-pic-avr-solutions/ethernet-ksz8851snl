/*
    \file   ksz8851snl.h

    \brief  KSZ8851SNL Ethernet controller driver header file.

    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

   THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER  
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

   IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

#ifndef __KSZ8851_H
#define __KSZ8851_H

// Byte enable bits for read and write
#define byteEnable0         (0x1)
#define byteEnable1         (0x2)
#define byteEnable2         (0x4)
#define byteEnable3         (0x8)
#define wordEnableOdd       (0x3)
#define wordEnableEven      (0xC)
#define dwordEnable         (0xF)
#define LSBMask             (0x03)
#define NO_QUEUED_PACKET    (0)
#define TX_BUFFER_FREE      (0x1FFF)

#define CCR_EEPROM_PRESENCE                 (0x0200)
#define CCR_SPI_BUS_MODE                    (0x0100)
#define CCR_32PIN_CHIP_PACKAGE              (0x0001)

#define OBCR_OUTPUT_PIN_DRIVE_STRENGTH      (0x0040)
#define OBCR_ONCHIP_BUS_CLOCK_SELECTION     (0X0004)
#define OBCR_ONCHIP_BUS_CLOCK_DIVIDER1      (0x0002)
#define OBCR_ONCHIP_BUS_CLCOK_DIVIDER0      (0x0001)

#define EEPCR_EESRWA                        (0x0020)
#define EEPCR_EESA                          (0x0010)
#define EEPCR_EESB                          (0x0008)
#define EEPCR_EECB2                         (0x0004)
#define EEPCR_EECB1                         (0x0002)
#define EEPCR_EECB0                         (0x0001)

#define MBIR_TXMBF                          (0x1000)
#define MBIR_TXMBFA                         (0x0800)
#define MBIR_TXMBFC2                        (0x0400)
#define MBIR_TXMBFC1                        (0x0200)
#define MBIR_TXMBFC0                        (0x0100)
#define MBIR_RXMBF                          (0x0010)
#define MBIR_RXMBFA                         (0x0008)
#define MBIR_RXMBFC2                        (0x0004)
#define MBIR_RXMBFC1                        (0x0002)
#define MBIR_RXMBFC0                        (0x0001)

#define GRR_QMU_MODULE_SOFT_RESET           (0x0002)
#define GRR_GLOBAL_SOFT_RESET               (0x0001)

#define WFCR_MPRXE                          (0x0080)
#define WFCR_WF3E                           (0x0008)
#define WFCR_WF2E                           (0x0004)
#define WFCR_WF1E                           (0x0002)
#define WFCR_WF0E                           (0x0001)

#define TXCR_TCGICMP                        (0x0100)
#define TXCR_TCGTCP                         (0x0040)
#define TXCR_TCGIP                          (0x0020)
#define TXCR_FTXQ                           (0x0010)
#define TXCR_TXFCE                          (0x0008)
#define TXCR_TXPE                           (0x0004)
#define TXCR_TXCE                           (0x0002)
#define TXCR_TXE                            (0x0001)

#define TXSR_TXLC                           (0x2000)
#define TXSR_TXMC                           (0x1000)
#define TXSR_TXFID5                         (0x0020)
#define TXSR_TXFID4                         (0x0010)
#define TXSR_TXFID3                         (0x0008)
#define TXSR_TXFID2                         (0x0004)
#define TXSR_TXFID1                         (0x0002)
#define TXSR_TXFID0                         (0x0001)

#define RXCR1_FRXQ                          (0x8000)
#define RXCR1_RXUDPFCC                      (0x4000)
#define RXCR1_RXTCPFCC                      (0x2000)
#define RXCR1_RXIPFCC                       (0x1000)
#define RXCR1_RXPAFMA                       (0x0800)
#define RXCR1_RXFCE                         (0x0400)
#define RXCR1_RXEFE                         (0x0200)
#define RXCR1_RXMAFMA                       (0x0100)
#define RXCR1_RXBE                          (0x0080)
#define RXCR1_RXME                          (0x0040)
#define RXCR1_RXUE                          (0x0020)
#define RXCR1_RXAE                          (0x0010)
#define RXCR1_RXINVF                        (0x0002)
#define RXCR1_RXE                           (0x0001)

#define RXCR2_SRDBL2                        (0x0080)
#define RXCR2_SRDBL1                        (0x0040)
#define RXCR2_SRDBL0                        (0x0020)
#define RXCR2_IUFFP                         (0x0010)
#define RXCR2_RXIUFCEZ                      (0x0008)
#define RXCR2_UDPLFE                        (0x0004)
#define RXCR2_RXICMPFCC                     (0x0002)
#define RXCR2_RXSAF                         (0x0001)

#define TXMIR_TXMA_MASK                     (0x1FFF)

#define RXFHSR_RXFV                         (0x8000)
#define RXFHSR_RXICMPFCS                    (0x2000)
#define RXFHSR_RXIPFCS                      (0x1000)
#define RXFHSR_RXTCPFCS                     (0x0800)
#define RXFHSR_RXUDPFCS                     (0x0400)
#define RXFHSR_RXBF                         (0x0080)
#define RXFHSR_RXMF                         (0x0040)
#define RXFHSR_RXUF                         (0x0020)
#define RXFHSR_RXMR                         (0x0010)
#define RXFHSR_RXFT                         (0x0008)
#define RXFHSR_RXFTL                        (0x0004)
#define RXFHSR_RXRF                         (0x0002)
#define RXFHSR_RXCE                         (0x0001)

#define RXFHBCR_RXBC                        (0x0FFF)

#define TXQCR_AETFE                         (0x0004)
#define TXQCR_TXQMAM                        (0x0002)
#define TXQCR_METFE                         (0x0001)

#define RXQCR_RXDTTS                        (0x1000)
#define RXQCR_RXDBCTS                       (0x0800)
#define RXQCR_RXFCTS                        (0x0400)
#define RXQCR_RXIPHTOE                      (0x0200)
#define RXQCR_RXDTTE                        (0x0080)
#define RXQCR_RXDBCTE                       (0x0040)
#define RXQCR_RXFCTE                        (0x0020)
#define RXQCR_ADRFE                         (0x0010)
#define RXQCR_SDA                           (0x0008)
#define RXQCR_RRXEF                         (0x0001)

#define TXFDPR_TXFPAI                       (0x4000)

#define RXFDPR_RXFPAI                       (0x4000)

#define RXDTTR_VALUE                        (0x03E8)
#define RXDBCTR_VALUE                       (0x1000)
#define RXFCT_VALUE                         (0X0001)

#define FCLWR_VALUE                         (0x0800)
#define FCHWR_VALUE                         (0x0400)

#define IER_LCIE                            (0x8000)
#define IER_TXIE                            (0x4000)
#define IER_RXIE                            (0x2000)
#define IER_RXOIE                           (0x0800)
#define IER_TXPSIE                          (0x0200)
#define IER_RXPSIE                          (0x0100)
#define IER_TXSAIE                          (0x0040)
#define IER_RXWFDIE                         (0x0020)
#define IER_RXMPDIE                         (0x0010)
#define IER_LDIE                            (0x0008)
#define IER_EDIE                            (0x0004)
#define IER_SPIBEIE                         (0x0002)
#define IER_DEDIE                           (0x0001)

#define ISR_LCIS                            (0x8000)
#define ISR_TXIS                            (0x4000)
#define ISR_RXIS                            (0x2000)
#define ISR_RXOIS                           (0x0800)
#define ISR_TXPSIS                          (0x0200)
#define ISR_RXPSIS                          (0x0100)
#define ISR_TXSAIS                          (0x0040)
#define ISR_RXWFDIS                         (0x0020)
#define ISR_RXMPDIS                         (0x0010)
#define ISR_LDIS                            (0x0008)
#define ISR_EDIS                            (0x0004)
#define ISR_SPIBEIS                         (0x0002)
#define ISR_CLEAR_ALL                       (0xFFFF)


#define CIDER_CHIP_ID                       (0x0080)

#define CGCR_LEDSEL0                        (0x0200)

#define IACR_READ_ENABLE                    (0x1000)
#define IACR_TABLE_SELECT1                  (0x0800)
#define IACR_TABLE_SELECT0                  (0x0400)
#define IACR_INDIRECT_ACCESS4               (0x0010)
#define IACR_INDIRECT_ACCESS3               (0x0008)
#define IACR_INDIRECT_ACCESS2               (0x0004)
#define IACR_INDIRECT_ACCESS1               (0x0002)
#define IACR_INDIRECT_ACCESS0               (0x0001)

#define PMECR_PME_DELAY_ENABLE              (0x4000)
#define PMECR_PME_OUTPUT_POLARITY           (0x1000)
#define PMECR_WUP_FRAME_EN                  (0x0800)
#define PMECR_MAGIC_PACKET                  (0x0400)
#define PMECR_LINK_CHANGE_TO_UP             (0x0200)
#define PMECR_SIGNAL_ENERGY_DETECTED        (0x0100)
#define PMECR_AUTO_WAKEUP_ENABLE            (0x0080)
#define PMECR_WAKEUP_TO_NORMAl              (0x0040)
#define PMECR_WAKEUP_FRAME_EVENT            (0x0020)
#define PMECR_WAKEUP_MAGIC_PACKET           (0x0010)
#define PMECR_WAKEUP_LINK                   (0x0008)
#define PMECR_WAKEUP_ENERGY                 (0x0004)
#define PMECR_PME_MODE1                     (0x0002)
#define PMECR_PME_MODE0                     (0x0001)

#define PHYRR_PHY_RESET                     (0x0001)

#define P1MBCR_LOCAL_LOOPBACK               (0x4000)
#define P1MBCR_FORCE100                     (0x2000)
#define P1MBCR_AN_ENABLE                    (0x1000)
#define P1MBCR_RESTART_AN                   (0x0200)
#define P1MBCR_FORCE_FULL_DUPLEX            (0x0100)
#define P1MBCR_HP_MDIX                      (0x0020)
#define P1MBCR_FORCE_MDIX                   (0x0010)
#define P1MBCR_DISABLE_MDIX                 (0x0008)
#define P1MBCR_DISABLE_TRANSMIT             (0x0002)
#define P1MBCR_DISABLE_LED                  (0x0001)

#define P1MBSR_T4_CAPABLE                   (0x8000)
#define P1MBSR_100_FULL_CAPABLE             (0x4000)
#define P1MBSR_100_HALF_CAPABLE             (0x2000)
#define P1MBSR_10_FULL_CAPABLE              (0x1000)
#define P1MBSR_10_HALF_CAPABLE              (0x0800)
#define P1MBSR_PREAMBLE_SUPPRESSED          (0x0040)
#define P1MBSR_AN_COMPLETE                  (0x0020)
#define P1MBSR_AN_CAPABLE                   (0x0008)
#define P1MBSR_LINK_STATUS                  (0x0004)
#define P1MBSR_JABBER_TEST                  (0x0002)
#define P1MBSR_EXTENDED_CAPABLE             (0x0001)

#define P1ANAR_NEXT_PAGE                    (0x8000)
#define P1ANAR_REMOTE_FAULT                 (0x2000)
#define P1ANAR_PAUSE                        (0x0400)
#define P1ANAR_ADV_100_FULL                 (0x0100)
#define P1ANAR_ADV_100_HALF                 (0x0080)
#define P1ANAR_ADV_10_FULL                  (0x0040)
#define P1ANAR_ADV_10_HALF                  (0x0020)

#define P1ANLPR_NEXT_PAGE                   (0x8000)
#define P1ANLPR_LP_ACK                      (0x4000)
#define P1ANLPR_REMOTE_FAULT                (0x2000)
#define P1ANLPR_PAUSE                       (0x0400)
#define P1ANLPR_ADV_100_FULL                (0x0100)
#define P1ANLPR_ADV_100_HALF                (0x0080)
#define P1ANLPR_ADV_10_FULL                 (0x0040)
#define P1ANLPR_ADV_10_HALF                 (0x0020)

#define P1SCLMD_VCT_RESULT1                 (0x4000)
#define P1SCLMD_VCT_RESULT0                 (0x2000)
#define P1SCLMD_VCT_ENABLE                  (0x1000)
#define P1SCLMD_FORCE_LINK                  (0x0800)
#define P1SCLMD_REMOTE_LOOPBACK             (0x0200)

#define P1CR_LED_OFF                        (0x8000)
#define P1CR_TXIDS                          (0x4000)
#define P1CR_RESTART_AN                     (0x2000)
#define P1CR_DISABLE_AUTO_MDI_MDIX          (0x0400)
#define P1CR_FORCE_MDIX                     (0x0200)
#define P1CR_AUTO_NEGOTIATION_ENABLE        (0x0080)
#define P1CR_FORCE_SPEED                    (0x0040)
#define P1CR_FORCE_DUPLEX                   (0x0020)
#define P1CR_ADVERTISED_FLOW_CONTROL_CAPABILITY         (0x0010)
#define P1CR_ADVERTISED_100BT_FULL_DUPLEX_CAPABILITY    (0x0008)
#define P1CR_ADVERTISED_100BT_HALF_DUPLEX_CAPABILITY    (0x0004)
#define P1CR_ADVERTISED_10BT_FULL_DUPLEX_CAPABILITY     (0x0002)
#define P1CR_ADVERTISED_10BT_HALF_DUPLEX_CAPABILITY     (0x0001)

#define P1SR_HP_MDIX                        (0x8000)
#define P1SR_POLARITY_REVERSE               (0x2000)
#define P1SR_OPERATION_SPEED                (0x0400)
#define P1SR_OPERATION_DUPLEX               (0x0200)
#define P1SR_MDIX_STATUS                    (0x0080)
#define P1SR_AN_DONE                        (0x0040)
#define P1SR_LINK_GOOD                      (0x0020)
#define P1SR_PARTNER_FLOW_CONTROL_CAPABILITY            (0x0010)
#define P1SR_PARTNER_100BT_FULL_DUPLEX_CAPABILITY       (0x0008)
#define P1SR_PARTNER_100BT_HALF_DUPLEX_CAPABILITY       (0x0004)
#define P1SR_PARTNER_10BT_FULL_DUPLEX_CAPABILITY        (0x0002)
#define P1SR_PARTNER_10BT_HALF_DUPLEX_CAPABILITY        (0x0001)

typedef enum
{
    read_io     = 0x00,
    write_io    = 0x40,
    fifo_read   = 0x80,
    fifo_write  = 0xC0,
}spi_inst_t;

typedef enum
{
    KSZ_CCR     = 0x08,
    KSZ_MARL    = 0x10,
    KSZ_MARM    = 0x12,
    KSZ_MARH    = 0x14,
    KSZ_OBCR    = 0x20,
    KSZ_EEPCR   = 0x22,
    KSZ_MBIR    = 0x24,
    KSZ_GRR     = 0x26,
    KSZ_WFCR    = 0x2A,
    KSZ_WF0CRC0 = 0x30,
    KSZ_WF0CRC1 = 0x32,
    KSZ_WF0BM0  = 0x34,
    KSZ_WF0BM1  = 0x36,
    KSZ_WF0BM2  = 0x38,
    KSZ_WF0BM3  = 0x3A,
    KSZ_WF1CRC0 = 0x40,
    KSZ_WF1CRC1 = 0x42,
    KSZ_WF1BM0  = 0x44,
    KSZ_WF1BM1  = 0x46,
    KSZ_WF1BM2  = 0x48,
    KSZ_WF1BM3  = 0x4A,
    KSZ_WF2CRC0 = 0x50,
    KSZ_WF2CRC1 = 0x52,
    KSZ_WF2BM0  = 0x54,
    KSZ_WF2BM1  = 0x56,
    KSZ_WF2BM2  = 0x58,
    KSZ_WF2BM3  = 0x5A,
    KSZ_WF3CRC0 = 0x60,
    KSZ_WF3CRC1 = 0x62,
    KSZ_WF3BM0  = 0x64,
    KSZ_WF3BM1  = 0x66,
    KSZ_WF3BM2  = 0x68,
    KSZ_WF3BM3  = 0x6A,
    KSZ_TXCR    = 0x70,
    KSZ_TXSR    = 0x72,
    KSZ_RXCR1   = 0x74,
    KSZ_RXCR2   = 0x76,
    KSZ_TXMIR   = 0x78,
    KSZ_RXFHSR  = 0x7C,
    KSZ_RXFHBCR = 0x7E,
    KSZ_TXQCR   = 0x80,
    KSZ_RXQCR   = 0x82,
    KSZ_TXFDPR  = 0x84, 
    KSZ_RXFDPR  = 0x86,
    KSZ_RXDTTR  = 0x8C,
    KSZ_RXDBCTR = 0x8E,
    KSZ_IER     = 0x90,
    KSZ_ISR     = 0x92,
    KSZ_RXFCT   = 0x9C,
    KSZ_RXFC    = 0x9D,
    KSZ_TXNTFSR = 0x9E,
    KSZ_MAHTR0  = 0xA0,
    KSZ_MAHTR1  = 0xA2,
    KSZ_MAHTR2  = 0xA4,
    KSZ_MAHTR3  = 0xA6,
    KSZ_FCLWR   = 0xB0,
    KSZ_FCHWR   = 0xB2,
    KSZ_FCOWR   = 0xB4,
    KSZ_CIDER   = 0xC0,
    KSZ_CGCR    = 0xC6,
    KSZ_IACR    = 0xC8,
    KSZ_IADLR   = 0xD0,
    KSZ_IADHR   = 0xD2,
    KSZ_PMECR   = 0xD4,
    KSZ_GSWUTR  = 0xD6,
    KSZ_PHYRR   = 0xD8,
    KSZ_P1MBCR  = 0xE4,
    KSZ_P1MBSR  = 0xE6,
    KSZ_PHY1ILR = 0xE8,
    KSZ_PHY1IHR = 0xEA,
    KSZ_P1ANAR  = 0xEC,
    KSZ_P1ANLPR = 0xEE,
    KSZ_P1SCLMD = 0xF4,
    KSZ_P1CR    = 0xF6,
    KSZ_P1SR    = 0xF8 ,            
}ksz8851_registers_t;

uint16_t controlWord;
uint16_t packetLength;
uint16_t pointerMask;

#endif // __KSZ8851_H