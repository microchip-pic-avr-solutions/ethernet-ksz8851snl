/**
  Ethernet Driver Header File
	
  Company:
    Microchip Technology Inc.

  File Name:
    ethernet_driver.h

  Summary:
    Public Interface definition for Ethernet driver.    

  Description:
  This files include definitions of public interface for an ethernet driver.
  See individual ethernet drivers for specific implementations of this interface.

 */

/*

©  [2015] Microchip Technology Inc. and its subsidiaries.  You may use this software 
and any derivatives exclusively with Microchip products. 
  
THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF 
NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS 
INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE 
IN ANY APPLICATION. 

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL 
OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED 
TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY 
OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S 
TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED 
THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS. 

*/

#ifndef ETHERNET_DRIVER_H
#define	ETHERNET_DRIVER_H

#include "tcpip_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "mac_address.h"

#ifdef KSZ8851SNL

#include "../../kszdriver/memory_buffer.h"

struct count
{
    uint16_t byteCount;
};

struct receivestatus
{
    unsigned rxBroadcast:1;
    unsigned rxMulticast:1;
    unsigned rxUnicast:1;
    unsigned rxMIIerror:1;
    unsigned rxFrametype:1;
    unsigned rxFrametoolong:1;
    unsigned rxRuntframe:1;
    unsigned rxCRCerror:1;
    unsigned rxFramevalid:1;
    unsigned :1;
    unsigned rxICMPcksmstatus:1;
    unsigned rxIPcksmstatus:1;
    unsigned rxTCPcksmstatus:1;
    unsigned rxUDPcksmstatus:1;
    unsigned :1;
    unsigned :1;
};
    
typedef union
{
    uint8_t v[4];
    struct receivestatus rxfhsr;
    struct count rxfhbcr;     
}receiveStatusVector_t;

#define ETH_ReadBlock(data,length)         BufferReadBlock(&dataBuffers.rxBuffer, data, length) 
#define ETH_Read8()                        BufferRead8(&dataBuffers.rxBuffer)
#define ETH_Read16()                       BufferRead16(&dataBuffers.rxBuffer)
#define ETH_Read32()                       BufferRead32(&dataBuffers.rxBuffer)
#define ETH_Write8(data)                   BufferWrite8(&dataBuffers.txBuffer, data)
#define ETH_Write16(data)                  BufferWrite16(&dataBuffers.txBuffer, data)
#define ETH_Write32(data)                  BufferWrite32(&dataBuffers.txBuffer, data)
#define ETH_WriteString(data)              BufferWriteString(&dataBuffers.txBuffer, data)
#define ETH_WriteBlock(data,length)        BufferWriteBlock(&dataBuffers.txBuffer, data, length)
#define ETH_Insert(data, length, offset)   BufferReplaceAtOffset(&dataBuffers.txBuffer, data, length, offset)
#define ETH_Dump(length)                   BufferDump(&dataBuffers.rxBuffer, length)
#define ETH_Copy(length)                   BufferRXtoTXCopy(&dataBuffers.rxBuffer, &dataBuffers.txBuffer, length)

#else

typedef union
{
    uint8_t v[4];
    struct
    {
        uint16_t byteCount;
        unsigned long_drop_event:1;
        unsigned :1;
        unsigned excessCarrierEvent:1;
        unsigned :1;
        unsigned crcError:1;
        unsigned lengthCheckError:1;
        unsigned lengthLongError:1;
        unsigned rxOK:1;
        unsigned rxMulticast:1;
        unsigned rxBroadcast:1;
        unsigned dribbleNibble:1;
        unsigned controlFrame:1;
        unsigned pauseControlFrame:1;
        unsigned unknownContrlFrame:1;
        unsigned vlanTagPresent:1;
        unsigned zero:1;
    };
}receiveStatusVector_t;

// Read functions for data
uint16_t ETH_ReadBlock(void*, uint16_t); // read a block of data from the MAC
uint8_t ETH_Read8(void);                 // read 1 byte from the MAC
uint16_t ETH_Read16(void);               // read 2 bytes and return them in host order
uint32_t ETH_Read24(void);              // read 3 bytes and return them in host order
uint32_t ETH_Read32(void);               // read 4 bytes and return them in host order
void ETH_Dump(uint16_t);                 // drop N bytes from a packet (data is lost)


uint16_t ETH_WriteString(const char *string);                            // write a string of data into the MAC
uint16_t ETH_WriteBlock(const char *, uint16_t);                         // write a block of data into the MAC    jira:M8TS-608
void ETH_Write8(uint8_t);                                          // write a byte into the MAC
void ETH_Write16(uint16_t);                                        // write 2 bytes into the MAC in Network order
void ETH_Write24(uint32_t data);                                   // write 3 bytes into the MAC in Network order
void ETH_Write32(uint32_t);                                        // write 4 bytes into the MAC in Network order
void ETH_Insert(char *,uint16_t, uint16_t);                        // insert N bytes into a specific offset in the TX packet
error_msg ETH_Copy(uint16_t);                                      // copy N bytes from saved read location into the current tx location

uint16_t ETH_GetWritePtr();
void ETH_ResetReadPtr();               //Reset the receive pointer to the Init
uint16_t ETH_GetStatusVectorByteCount(void);
void ETH_SetStatusVectorByteCount(uint16_t);
uint16_t ETH_ReadSavedWRPT(void);
void ETH_SaveWRPT(void);
void ETH_SetRxByteCount(uint16_t count);
uint16_t ETH_GetRxByteCount(void);
void ETH_TxReset(void);
void ETH_MoveBackReadPtr(uint16_t offset);

#endif	/* KSZ8851SNL */

typedef struct 
{
	unsigned error:1;
	unsigned pktReady:1;
	unsigned up:1;
	unsigned idle:1;
	unsigned linkChange:1;
    unsigned bufferBusy:1;  
    unsigned :3;
    uint16_t saveRDPT;
    uint16_t saveWRPT;
} ethernetDriver_t;

typedef struct 
{
    uint16_t flags;
    uint16_t packetStart;
    uint16_t packetEnd;

    void    *prevPacket;
    void    *nextPacket;
} txPacket_t;

extern volatile ethernetDriver_t ethData;

#define ETH_packetReady() ethData.pktReady
#define ETH_linkCheck()   ethData.up
#define ETH_linkChanged() ethData.linkChange

void ETH_CloseSPI(void);           // Release SPI Bus
void ETH_OpenSPI(void);            // Open SPI Bus

void ETH_Init(void);            // setup the ethernet and get it running
void ETH_EventHandler(void);    // Manage the MAC events.  Poll this or put it in the ISR
void ETH_NextPacketUpdate(void);    // Update the pointers for the next available RX packets

void ETH_ResetReceiver(void);   // Reset the receiver
void ETH_SendSystemReset(void); // Reset the transmitte    
void ETH_Flush(void);                    // drop the rest of this packet and release the buffer
uint16_t ETH_GetFreeTxBufferSize(void);                         // returns the available space size in the TX buffer

error_msg ETH_WriteStart(const mac48Address_t *dest_mac, uint16_t type);
error_msg ETH_Send(void);                                          // Send the TX packet
uint16_t ETH_TxComputeChecksum(uint16_t position, uint16_t len, uint16_t seed); // compute the checksum of len bytes starting with position.
uint16_t ETH_RxComputeChecksum(uint16_t len, uint16_t seed);

void ETH_GetMAC(uint8_t *);            // get the MAC address
void ETH_SetMAC(uint8_t *);            // set the MAC address
void ETH_SaveRDPT(void);               // save the receive pointer for copy
uint16_t ETH_GetReadPtr(void);         //returns the value of the read pointer
void ETH_SetReadPtr(uint16_t);         //sets the read pointer to a specific address
void ETH_ResetByteCount(void);
uint16_t ETH_GetByteCount(void);

bool ETH_CheckLinkUp(void);
#endif	/* ETHERNET_DRIVER_H */
