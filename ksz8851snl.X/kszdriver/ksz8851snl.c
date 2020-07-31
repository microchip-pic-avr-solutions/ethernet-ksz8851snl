/*
    \file   ksz8851snl.c

    \brief  KSZ8851SNL Ethernet controller driver source file.

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
    

#include "../mcc_generated_files/mcc.h"
#include "ksz8851snl.h"
#include <stdint.h>
#include <stddef.h>
#include "memory_buffer.h"
#include "../mcc_generated_files/TCPIPLibrary/ethernet_driver.h"
#include "../mcc_generated_files/spi1_types.h"
#include "../mcc_generated_files/drivers/spi_master.h"
#include "../mcc_generated_files/TCPIPLibrary/network.h"

// Packet write in progress, not ready for transmit
#define ETH_WRITE_IN_PROGRESS           (0x0001 << 0)
// Packet complete, in queue for transmit
#define ETH_TX_QUEUED                   (0x0001 << 1)
// Flag for pool management - free or allocated
#define ETH_ALLOCATED                   (0x0001 << 2)

// adjust these parameters for the MAC...
#define RAMSIZE                         (0x47FF) //18431
#define MAX_TX_PACKET_SIZE              (1518)
#define MIN_TX_PACKET_SIZE              (64)
#define MAX_TX_PACKETS                  (20)

#define TX_STATUS_VECTOR_SIZE           (4)

// typical memory map for the MAC buffers
#define TXSTART                         (0x0000)
#define TXEND                           (0x1800) // 6144
#define RXSTART                         (0x0000) 
#define RXEND                           (0x2FFF) //12287

#define MIN_TX_PACKET                   (MIN_TX_PACKET_SIZE + TX_STATUS_VECTOR_SIZE)
#define TX_BUFFER_SIZE                  (TXEND  - TXSTART) //6144

#define TX_BUFFER_MID                   ((TXSTART + TXEND) >> 1 ) //3072

#define SetBit( bitField, bitMask )     do{ bitField = bitField | bitMask; } while(0)
#define ClearBit( bitField, bitMask )   do{ bitField = bitField & (~bitMask); } while(0)
#define CheckBit( bitField, bitMask )   (bool)(bitField & bitMask)

// Define a temporary register for passing data to inline assembly
// This is to work around the 110110 LSB errata and to control RDPTR WRPTR update counts

static txPacket_t  *pHead;
static txPacket_t  *pTail;
static txPacket_t txData[MAX_TX_PACKETS];
static receiveStatusVector_t rxPacketStatusVector;
static uint16_t nextPacketPointer;
static uint8_t ethListSize;
volatile ethernetDriver_t ethData;
const mac48Address_t *eth_MAC;

static error_msg ETH_SendQueued(void);
static void ETH_PacketListReset(void);
static txPacket_t* ETH_NewPacket(void);
static void ETH_RemovePacket(txPacket_t* packetHandle);

static uint8_t KSZ88_RegRead8(ksz8851_registers_t);
static uint16_t KSZ88_RegRead16(ksz8851_registers_t);
static uint32_t KSZ88_RegRead32(ksz8851_registers_t);

static void KSZ88_RegWrite8(ksz8851_registers_t, uint8_t);
static void KSZ88_RegWrite16(ksz8851_registers_t, uint16_t);
static void KSZ88_RegWrite32(ksz8851_registers_t, uint32_t);

static void KSZ88_BitFieldSet(ksz8851_registers_t, uint16_t);
static void KSZ88_BitFieldClear(ksz8851_registers_t, uint16_t);

/*******************************************************************************/

bool ETH_CheckLinkUp(void)
{
    uint16_t p1sr;
    
    p1sr = KSZ88_RegRead16(KSZ_P1SR);
    
    if(p1sr & P1SR_LINK_GOOD)
    {
        ethData.up = true;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * "Allocate" a new packet element and link it into the chained list
 * @param 
 * @return  packet address
 */
static txPacket_t* ETH_NewPacket(void)
{
    uint8_t index = 0;

    if( ethListSize == MAX_TX_PACKETS )
    {
        return NULL;
    }

    while( index < MAX_TX_PACKETS )
    {
        if( CheckBit(txData[index].flags, ETH_ALLOCATED) == false )
        {
            // reset all flags
            txData[index].flags = 0;                        
            // allocated = true - mark the handle as allocated
            SetBit(txData[index].flags, ETH_ALLOCATED);     

            txData[index].packetEnd = TXEND;

            txData[index].prevPacket = NULL;
            txData[index].nextPacket = pHead;

            if( pHead != NULL )
            {
                pHead->prevPacket = &txData[index];
                txData[index].packetStart = pHead->packetEnd + TX_STATUS_VECTOR_SIZE;
                    
                // Try to keep a 2byte alignment
                if( txData[index].packetStart & 0x0001 )
                {
                    // Make sure the end of the packet is odd, so the beginning of the next one is even
                    ++ txData[index].packetStart;
                }
            }
            else
            {
                txData[index].packetStart = TXSTART;
                pTail = (txPacket_t*)&txData[index];
            }

            pHead = (txPacket_t*)&txData[index];

            ethListSize ++;
            return (&txData[index]);
        }
        index ++;
    }
    return NULL;
}

/**
 * Reset the Ethernet Packet List
 * @param 
 * @return
 */
static void ETH_PacketListReset(void)
{
    uint16_t index = 0;
    uint8_t* ptr = (uint8_t*)&txData;
    ethListSize = 0;

    pHead = NULL;
    pTail = NULL;

    while( index < (MAX_TX_PACKETS * sizeof(txPacket_t)) )
    {
        ptr[index] = 0;
        index++;
    }
}

/**
 * Unlink a packet element from the chained list and "deallocate" it
 * @param   packetHandle
 * @return 
 */
static void ETH_RemovePacket(txPacket_t* pPacket)
{    
#ifdef VALIDATE_ALLOCATED_PTR
    uint8_t index = 0;
#endif /* VALIDATE_ALLOCATED_PTR */

    if( (pPacket == NULL) || (ethListSize == 0) )
    {
        return;
    }

#ifdef VALIDATE_ALLOCATED_PTR
    while( index < MAX_TX_PACKETS )
    {
        if( (pPacket == &txData[index]) && (txData[index].allocated == true) )
        {
            break;
        }
        index ++;
    }
    if( index == MAX_TX_PACKETS )
    {
        return;
    }
#endif  /* VALIDATE_ALLOCATED_PTR */

    // Unlink from the chained list
    if( pPacket->nextPacket == NULL )
    {
        pTail = pPacket->prevPacket;
        if( pTail != NULL )
        {
            pTail->nextPacket = NULL;
        }
    }

    if( pPacket->prevPacket == NULL )
    {
        pHead = pPacket->nextPacket;
        if( pHead != NULL )
        {
            pHead->prevPacket = NULL;
        }
    }

    // Deallocate
    pPacket->flags = 0;
    pPacket->prevPacket = NULL;
    pPacket->nextPacket = NULL;

    ethListSize --;

    return;
}

/**
 * Release SPI Bus
 */
void ETH_CloseSPI(void)
{
    spi1_close();
}
 
/**
  * Connect SPI Bus
  */
void ETH_OpenSPI(void)
{
    while (!spi_master_open(MAC));
}

/**
 * Ethernet Initialization - Initializes TX/RX Buffer, MAC and PHY
 */
void ETH_Init(void)
{
    ETH_OpenSPI();

   // initialize the driver variables
    ethData.error = false; // no error
    ethData.up = false; // no link
    ethData.linkChange = false;
    
    ETH_PacketListReset();

    ethData.saveRDPT = 0;
    controlWord = 0x0080;
    packetLength = 0x0000;
    pointerMask = 0x07FF;
    
    KSZ88_BitFieldSet(KSZ_PMECR, PMECR_WAKEUP_TO_NORMAl);
    // Software reset
    ETH_SendSystemReset(); 

    eth_MAC =  MAC_getAddress();
        
    KSZ88_RegWrite16(KSZ_MARH, ((eth_MAC->mac_array[0] << 8) | (eth_MAC->mac_array[1]))); NOP();
    KSZ88_RegWrite16(KSZ_MARM, ((eth_MAC->mac_array[2] << 8) | (eth_MAC->mac_array[3]))); NOP();
    KSZ88_RegWrite16(KSZ_MARL, ((eth_MAC->mac_array[4] << 8) | (eth_MAC->mac_array[5]))); NOP();

    KSZ88_RegWrite16(KSZ_TXFDPR, TXFDPR_TXFPAI);
    KSZ88_RegWrite16(KSZ_TXCR, TXCR_TXFCE | TXCR_TXPE | TXCR_TXCE);

    KSZ88_RegWrite16(KSZ_RXFDPR, RXFDPR_RXFPAI);
    KSZ88_RegWrite16(KSZ_RXCR1, RXCR1_RXUDPFCC | RXCR1_RXTCPFCC | RXCR1_RXIPFCC | RXCR1_RXPAFMA | RXCR1_RXFCE | RXCR1_RXBE | RXCR1_RXUE);
    KSZ88_RegWrite16(KSZ_RXCR2, RXCR2_SRDBL2 | RXCR2_IUFFP | RXCR2_RXIUFCEZ | RXCR2_UDPLFE | RXCR2_RXICMPFCC);
    KSZ88_RegWrite16(KSZ_RXQCR,RXQCR_RXIPHTOE | RXQCR_RXFCTE | RXQCR_ADRFE);
    
    KSZ88_RegWrite16(KSZ_RXDTTR, RXDTTR_VALUE);
    KSZ88_RegWrite16(KSZ_RXDBCTR, RXDBCTR_VALUE);
    
    KSZ88_RegWrite8(KSZ_RXFCT, RXFCT_VALUE);
    KSZ88_RegWrite16(KSZ_FCLWR, FCLWR_VALUE);
    KSZ88_RegWrite16(KSZ_FCHWR, FCHWR_VALUE);
    KSZ88_RegWrite16(KSZ_ISR, ISR_CLEAR_ALL);

    KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
    KSZ88_BitFieldSet(KSZ_TXCR, TXCR_TXE);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_RXE);
}

/**
 * Poll Ethernet Controller for new events
 */
void ETH_EventHandler(void)
{
    uint16_t isr;
       
    isr = KSZ88_RegRead16(KSZ_ISR);
    if((isr & ISR_LDIS) > 0)
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_LDIS);           
        KSZ88_BitFieldSet(KSZ_PMECR, PMECR_WAKEUP_LINK);   
    }
    if((isr & ISR_LCIS) > 0)
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_LCIS);            
        ethData.linkChange = true;
        ethData.up = false;
        ETH_CheckLinkUp();
    }
    
    if((isr & ISR_TXIS) > 0)
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_TXIS);            
        if( ethListSize > 0 )
        {
            //Send the next queued packet
            ETH_SendQueued();
        }
    }
    
    if((isr & ISR_RXIS) > 0)
    {
        KSZ88_BitFieldSet(KSZ_ISR, ISR_RXIS);
        if(ethData.pktReady == false)
        {
            ethData.pktReady = true;
        }          
    }
    KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
}

/**
 * Retrieve the packet and write it to software buffer
 */
void ETH_NextPacketUpdate(void)
{
    uint8_t rxfc_val;
    uint16_t rxstat_val;
    uint16_t rxlen_val;

    rxfc_val = KSZ88_RegRead8(KSZ_RXFC);
    // Disabling all interrupts before reading a packet
    KSZ88_RegWrite16(KSZ_IER, 0x0000);
    rxstat_val = KSZ88_RegRead16(KSZ_RXFHSR);
    rxlen_val = KSZ88_RegRead16(KSZ_RXFHBCR);
    rxlen_val = rxlen_val & 0xfff;
    
    //Initialize the RX buffer
    RXBuffer_Init();
    KSZ88_BitFieldClear(KSZ_RXFDPR, pointerMask);
    KSZ88_BitFieldSet(KSZ_RXQCR, RXQCR_SDA);
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(fifo_read);
    ETH_SPI_READ8();
    ETH_SPI_READ8();
    ETH_SPI_READ8();
    ETH_SPI_READ8();       
    ((char *) &rxPacketStatusVector)[0] = ETH_SPI_READ8(); 
    ((char *) &rxPacketStatusVector)[1] = ETH_SPI_READ8();
    ((char *) &rxPacketStatusVector)[2] = ETH_SPI_READ8();
    ((char *) &rxPacketStatusVector)[3] = ETH_SPI_READ8();
    ETH_SPI_READ8();
    ETH_SPI_READ8(); 
    
    // Received frame length = RXFHBCR minus 4-byte CRC and 2-byte offset enable
    BufferReceive(&dataBuffers.rxBuffer, (rxlen_val-6));
    ETH_NCS_HIGH();
    KSZ88_BitFieldClear(KSZ_RXQCR, RXQCR_SDA);
    // Enabling the interrupts after reading a packet from RX FIFO
    KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
}

/**
 * Read 1 byte from KSZ Register
 * @param address
 * @return
 */
static uint8_t KSZ88_RegRead8(ksz8851_registers_t address)
{
    uint8_t value;
    uint16_t cmd;
    uint8_t cmd_byte0;
    uint8_t cmd_byte1;
    
    //Checking the register address to add the correct mask value
    if((address & LSBMask) == 0x00) 
    {
        cmd = ((read_io << 8)| (byteEnable0 << 10)| (address << 2));
    }
    else if((address & LSBMask) == 0x01)
    {
        cmd = ((read_io << 8)| (byteEnable1 << 10)| (address << 2));
    }
    else if((address & LSBMask) == 0x02)
    {
        cmd = ((read_io << 8)| (byteEnable2 << 10)| (address << 2));
    }
    else
    {
        cmd = ((read_io << 8)| (byteEnable3 << 10)| (address << 2));
    }
    
    cmd_byte0 = (cmd >> 8);
    cmd_byte1 = (cmd & 0xFF);
        
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(cmd_byte0);
    ETH_SPI_WRITE8(cmd_byte1);
    value = ETH_SPI_READ8();
    ETH_NCS_HIGH();
    
    return value;   
}

/**
 * Read 2 byte from KSZ Register
 * @param address
 * @return
 */

static uint16_t KSZ88_RegRead16(ksz8851_registers_t address)
{
    uint16_t value;
    uint16_t cmd;
    uint8_t cmd_byte0;
    uint8_t cmd_byte1;
    
    //Checking to find if the register address is even or odd to use the correct mask value
    if(address & 0x02) 
    {
        cmd = ((read_io << 8)| (wordEnableEven << 10)| (address << 2));
    }
    else
    {
        cmd = ((read_io << 8)| (wordEnableOdd << 10)| (address << 2));
    }
    
    cmd_byte0 = (cmd >> 8);
    cmd_byte1 = (cmd & 0xFF);
            
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(cmd_byte0);
    ETH_SPI_WRITE8(cmd_byte1);
    ((char *) &value)[0] = ETH_SPI_READ8();
    ((char *) &value)[1] = ETH_SPI_READ8();
    ETH_NCS_HIGH();
    
    return value;
}

/**
 * Read 4 byte from KSZ Register
 * @param address
 * @return
 */
static uint32_t KSZ88_RegRead32(ksz8851_registers_t address)
{
    uint32_t value;
    uint16_t cmd;
    uint8_t cmd_byte0;
    uint8_t cmd_byte1;
    
    cmd = ((read_io << 8)| (dwordEnable << 10)| (address << 2));
    
    cmd_byte0 = (cmd >> 8);
    cmd_byte1 = (cmd & 0xFF);
        
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(cmd_byte0);
    ETH_SPI_WRITE8(cmd_byte1);
    ((char *) &value)[0] = ETH_SPI_READ8();
    ((char *) &value)[1] = ETH_SPI_READ8();
    ((char *) &value)[2] = ETH_SPI_READ8();
    ((char *) &value)[3] = ETH_SPI_READ8();
    ETH_NCS_HIGH();
    
    return value;
}

/**
 * Write 1 byte to KSZ Register
 * @param address
 * @param value
 */
static void KSZ88_RegWrite8(ksz8851_registers_t address, uint8_t value)
{
    uint16_t cmd;
    uint8_t cmd_byte0;
    uint8_t cmd_byte1;
    
    //Checking the register address to add the correct mask value
    if((address & LSBMask) == 0x00) 
    {
        cmd = ((write_io << 8)| (byteEnable0 << 10)| (address << 2));
    }
    else if((address & LSBMask) == 0x01)
    {
        cmd = ((write_io << 8) | (byteEnable1 << 10)| (address << 2));
    }
    else if((address & LSBMask) == 0x02)
    {
        cmd = ((write_io << 8)| (byteEnable2 << 10)| (address << 2));
    }
    else
    {
        cmd = ((write_io << 8)| (byteEnable3 << 10)| (address << 2));
    }
  
    cmd_byte0 = (cmd >> 8);
    cmd_byte1 = (cmd & 0xFF);
        
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(cmd_byte0);
    ETH_SPI_WRITE8(cmd_byte1);
    ETH_SPI_WRITE8(value);
    ETH_NCS_HIGH();
}

/**
 * Write 2 byte to KSZ Register
 * @param address
 * @param value
 */
static void KSZ88_RegWrite16(ksz8851_registers_t address, uint16_t value)
{
    uint16_t cmd;
    uint8_t cmd_byte0;
    uint8_t cmd_byte1;
    
    //Checking to find if the register address is even or odd to use the correct mask value
    if(address & 0x02) 
    {
        cmd = ((write_io << 8)| (wordEnableEven << 10)| (address << 2));
    }
    else
    {
        cmd = ((write_io << 8)| (wordEnableOdd << 10)| (address << 2));
    }
    
    cmd_byte0 = (cmd >> 8);
    cmd_byte1 = (cmd & 0xFF);
    
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(cmd_byte0);
    ETH_SPI_WRITE8(cmd_byte1);
    ETH_SPI_WRITE8(((char *) &value)[0]);
    ETH_SPI_WRITE8(((char *) &value)[1]);
    ETH_NCS_HIGH();        
}

/**
 * Write 4 byte to KSZ Register
 * @param address
 * @param value
 */
static void KSZ88_RegWrite32(ksz8851_registers_t address, uint32_t value)
{
    uint16_t cmd;
    uint8_t cmd_byte0;
    uint8_t cmd_byte1;
    
    cmd = ((write_io << 8)| (dwordEnable << 10)| (address << 2));
    
    cmd_byte0 = (cmd >> 8);
    cmd_byte1 = (cmd & 0xFF);
    
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(cmd_byte0);
    ETH_SPI_WRITE8(cmd_byte1);
    ETH_SPI_WRITE8(((char *) &value)[0]);
    ETH_SPI_WRITE8(((char *) &value)[1]);
    ETH_SPI_WRITE8(((char *) &value)[2]);
    ETH_SPI_WRITE8(((char *) &value)[3]);
    ETH_NCS_HIGH(); 
}

/**
 * Bit Field Clear
 * @param address
 * @param value
 */
static void KSZ88_BitFieldClear(ksz8851_registers_t address, uint16_t value) 
{
    uint16_t temp;
    ETH_NCS_LOW();
    temp = KSZ88_RegRead16(address);
    temp = (temp & (~value));
    KSZ88_RegWrite16(address, temp);
    ETH_NCS_HIGH();
}

/**
 * Bit Field Set
 * @param address
 * @param value
 */
static void KSZ88_BitFieldSet(ksz8851_registers_t address, uint16_t value)
{
    uint16_t temp;
    ETH_NCS_LOW();
    temp = KSZ88_RegRead16(address);
    temp = temp | value;
    KSZ88_RegWrite16(address, temp);
    ETH_NCS_HIGH();   
}

/**
 * Returns the available space size in the Ethernet TX Buffer
 * @param 
 * @return available space left in the TX buffer
 */
uint16_t ETH_GetFreeTxBufferSize(void)
{
    uint16_t freespace = KSZ88_RegRead16(KSZ_TXFDPR);
    freespace &= TX_BUFFER_FREE; 
    return ((uint16_t)((uint16_t)TXEND - (freespace)));
}

/**
 * start a packet.
 * If the Ethernet transmitter is idle, then start a packet.  Return is SUCCESS if the packet was started.
 * @param dest_mac
 * @param type
 * @return SUCCESS if packet started.  BUFFER_BUSY or TX_LOGIC_NOT_IDLE if buffer or transmitter is busy respectively
 */
error_msg ETH_WriteStart(const mac48Address_t *dest_mac, uint16_t type)
{
    //Initialize the TX buffer
    TXBuffer_Init();
    txPacket_t* ethPacket = NULL;

    // Check for TX in progress
    if((KSZ88_RegRead16(KSZ_TXQCR) & 0x0001))
    {
        return TX_LOGIC_NOT_IDLE;
    }
    
     if( KSZ88_RegRead16(KSZ_TXMIR) < TX_BUFFER_MID)
    {
        return BUFFER_BUSY;
    }
       
    // Initialize a new packet handler. It is automatically placed in the queue
    ethPacket = (txPacket_t*)ETH_NewPacket();

    if( ethPacket == NULL )
    {
        // No more available packets
        return BUFFER_BUSY;
    }

    SetBit(ethPacket->flags, ETH_WRITE_IN_PROGRESS);    // writeInProgress = true;
    
    ETH_ResetByteCount(); 
    
    // Disabling all interrupts before starting a packet
    KSZ88_RegWrite16(KSZ_IER, 0x0000);
    
    BufferWrite16(&dataBuffers.txBuffer, controlWord);
    BufferWrite16(&dataBuffers.txBuffer, packetLength);
    BufferWrite8(&dataBuffers.txBuffer, dest_mac->mac_array[0]);
    BufferWrite8(&dataBuffers.txBuffer, dest_mac->mac_array[1]);
    BufferWrite8(&dataBuffers.txBuffer, dest_mac->mac_array[2]);
    BufferWrite8(&dataBuffers.txBuffer, dest_mac->mac_array[3]);
    BufferWrite8(&dataBuffers.txBuffer, dest_mac->mac_array[4]);
    BufferWrite8(&dataBuffers.txBuffer, dest_mac->mac_array[5]);
    BufferWrite8(&dataBuffers.txBuffer, eth_MAC->mac_array[0]);
    BufferWrite8(&dataBuffers.txBuffer, eth_MAC->mac_array[1]);
    BufferWrite8(&dataBuffers.txBuffer, eth_MAC->mac_array[2]);
    BufferWrite8(&dataBuffers.txBuffer, eth_MAC->mac_array[3]);
    BufferWrite8(&dataBuffers.txBuffer, eth_MAC->mac_array[4]);
    BufferWrite8(&dataBuffers.txBuffer, eth_MAC->mac_array[5]);
    BufferWrite16(&dataBuffers.txBuffer, type);
    
    return SUCCESS;
}


/**
 * Enqueue the latest written packet and start the transmission of a queued packet
 * @return
 */
static error_msg ETH_SendQueued(void)
{
    if((pHead->flags & ETH_TX_QUEUED) > NO_QUEUED_PACKET)
    {
        // "Close" the latest written packet and enqueue it
        // txQueued = false
        ClearBit( pHead->flags, ETH_TX_QUEUED);         
         
        //Start sending
        KSZ88_BitFieldSet(KSZ_TXQCR, TXQCR_METFE);   
        while(KSZ88_RegRead16(KSZ_TXQCR) & TXQCR_METFE);
        ETH_RemovePacket(pTail);
        return SUCCESS;
    }
    else
    {
        return BUFFER_BUSY;
    }
}

/**
 * Start the Transmission
 * @return
 */
error_msg ETH_Send(void)
{
    uint16_t packetEnd;
    
    KSZ88_BitFieldSet(KSZ_RXQCR, RXQCR_SDA);
    ETH_NCS_LOW();
    ETH_SPI_WRITE8(fifo_write);
    BufferSend(&dataBuffers.txBuffer);    
    ETH_NCS_HIGH();
    KSZ88_BitFieldClear(KSZ_RXQCR, RXQCR_SDA);
    
    // Enabling the interrupts after sending a packet
    KSZ88_RegWrite16(KSZ_IER, IER_LCIE | IER_TXIE | IER_RXIE | IER_LDIE);
    
    packetEnd = ((KSZ88_RegRead16(KSZ_TXFDPR)) & pointerMask) - 1;
    
    if (ethData.up == 0)
    {
        return LINK_NOT_FOUND;
    }

    if( ethListSize == 0 )
    {
        return BUFFER_BUSY;
    }
    
    // writeInProgress = false
    ClearBit( pHead->flags, ETH_WRITE_IN_PROGRESS);     
    pHead->packetEnd = packetEnd;
    
    // txQueued = true
    SetBit( pHead->flags, ETH_TX_QUEUED);               
   
    // The packet is prepared to be sent / queued at this time
    if( (KSZ88_RegRead16(KSZ_TXQCR) & TXQCR_METFE)|| (ethListSize > 1) )
    {
        return TX_QUEUED;
    }
    return ETH_SendQueued();
}

/**
 * Clears all bytes from the RX buffer
 */
void ETH_Flush(void)
{
    ethData.pktReady = false;
    KSZ88_BitFieldClear(KSZ_RXCR1, RXCR1_RXE);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_FRXQ);
    KSZ88_BitFieldClear(KSZ_RXCR1, RXCR1_FRXQ);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_RXE);   
}

void ETH_ResetReceiver(void)
{
    KSZ88_BitFieldClear(KSZ_RXCR1, RXCR1_RXE);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_FRXQ);
    KSZ88_BitFieldClear(KSZ_RXCR1, RXCR1_FRXQ);
    KSZ88_BitFieldSet(KSZ_RXCR1, RXCR1_RXE);
}

/**
 * System Software Reset
 */
void ETH_SendSystemReset(void)
{
    KSZ88_BitFieldSet(KSZ_GRR, GRR_GLOBAL_SOFT_RESET);
    NOP();
    KSZ88_BitFieldClear(KSZ_GRR, GRR_GLOBAL_SOFT_RESET);
    NOP();   
}

/**
 * Calculate the TX Checksum - Software Checksum
 * @param position
 * @param length
 * @param seed
 * @return
 */
uint16_t ETH_TxComputeChecksum(uint16_t position, uint16_t length, uint16_t seed)
{
    uint16_t i = 0;
    uint32_t checksum;
    uint16_t value = 0;

    checksum = seed;      
    position = position + sizeof(controlWord)+ sizeof(packetLength);
    for(i=0; i<length; i++)
    {
        while(length > 1)
        {
            value = 0;
            ((char *)&value)[1] = BufferPeekCopy( &dataBuffers.txBuffer, position);
            position++;
            ((char *)&value)[0] = BufferPeekCopy( &dataBuffers.txBuffer, position);
            position++;
            
            checksum += value;
            length -= 2;
        }
        if(length!=0)
        {
            value = 0;
            ((char *)&value)[1] = BufferPeekCopy( &dataBuffers.txBuffer, position);
            ((char *)&value)[0] = 0;
            checksum += value;
        }
    }
    
    // wrap the checksum if its greater than 16 bits
    while(checksum >> 16)
    {
        checksum = ((checksum & 0xFFFF) + (checksum>>16));
    }

    // invert the number.
    checksum = ~checksum;
    checksum = (uint16_t)checksum;
   
    checksum = htons(checksum);
    return (uint16_t)checksum;
}

/**
 * Calculate RX checksum - Software checksum
 * @param len
 * @param seed
 * @return
 */
uint16_t ETH_RxComputeChecksum(uint16_t len, uint16_t seed)
{
    uint32_t checksum;
    uint16_t value;
    uint16_t i;
    uint16_t offset = 0;
       
    checksum = seed;
    
    for(i=0; i<len; i++)
    {   
        while(len > 1)
        {
            value = 0;
            ((char *)&value)[1] = BufferPeekCopy( &dataBuffers.rxBuffer, offset);
            offset++;
            ((char *)&value)[0] = BufferPeekCopy( &dataBuffers.rxBuffer, offset);
            offset++;
            checksum += value;
            len -= 2;
        }
        if(len)
        {
            value = 0;
            ((char *)&value)[1] = BufferPeekCopy( &dataBuffers.rxBuffer, offset);
            ((char *)&value)[0] = 0;
            checksum += value;
        }
    }
    // wrap the checksum if its greater than 16 bits
    while(checksum >> 16)
    {
        checksum = ((checksum & 0xFFFF) + (checksum>>16));
    }
    
    // invert the number.
    checksum = ~checksum;
    checksum = (uint16_t)checksum;
    
    // Return the resulting checksum
    return (uint16_t)((checksum & 0xFF00) >> 8) | ((checksum & 0x00FF) << 8);
}

/**
 * To get the MAC address
 * @param mac
 */
void ETH_GetMAC(uint8_t *macAddr)
{
   *macAddr++ = ((KSZ88_RegRead16(KSZ_MARH) >> 8) & 0xFF);
   *macAddr++ = ((KSZ88_RegRead16(KSZ_MARH) & 0xFF));
   *macAddr++ = ((KSZ88_RegRead16(KSZ_MARM) >> 8) & 0xFF);
   *macAddr++ = ((KSZ88_RegRead16(KSZ_MARM) & 0xFF));
   *macAddr++ = ((KSZ88_RegRead16(KSZ_MARL) >> 8) & 0xFF);
   *macAddr++ = ((KSZ88_RegRead16(KSZ_MARL) & 0xFF));
}

/**
 * To set the MAC address
 * @param mac
 */
void ETH_SetMAC(uint8_t *macAddr)
{
    KSZ88_RegWrite16(KSZ_MARH, ((eth_MAC->mac_array[0] << 8) | (eth_MAC->mac_array[1])));
    KSZ88_RegWrite16(KSZ_MARM, ((eth_MAC->mac_array[2] << 8) | (eth_MAC->mac_array[3])));
    KSZ88_RegWrite16(KSZ_MARL, ((eth_MAC->mac_array[4] << 8) | (eth_MAC->mac_array[5])));     
}

void ETH_SaveRDPT(void)
{
    ethData.saveRDPT = (volatile uint16_t)dataBuffers.rxBuffer.currentLocation;
}

uint16_t ETH_GetReadPtr(void)
{
    return (uint16_t)dataBuffers.rxBuffer.currentLocation;
}

void ETH_SetReadPtr(uint16_t rdptr)
{  
    dataBuffers.rxBuffer.currentLocation = rdptr;
}

void ETH_ResetByteCount(void)
{
    ethData.saveWRPT = dataBuffers.txBuffer.dataLength;
}

uint16_t ETH_GetByteCount(void)
{
    return (dataBuffers.txBuffer.dataLength - ethData.saveWRPT);
}




