/*
    \file   memory_buffer.h

    \brief  KSZ8851SNL ethernet controller buffer header file.

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

#ifndef MEMORY_BUFFER_H
#define	MEMORY_BUFFER_H

#include <stdint.h>
#include <string.h>
#include "../mcc_generated_files/spi1_types.h"
#include "../mcc_generated_files/drivers/spi_master.h"
#include "../mcc_generated_files/TCPIPLibrary/tcpip_types.h"

#define TXBUFFER_SIZE       (1700)
#define RXBUFFER_SIZE       (1700)

#define ETH_NCS_HIGH()      do{LATD3 = 1;} while(0)  //Use the Ethernet Chip select as per your hardware specification here
#define ETH_NCS_LOW()       do{LATD3 = 0;} while(0)  //Use the Ethernet Chip select as per your hardware specification here
        
#define ETH_SPI_READ8()     spi1_exchangeByte(0)
#define ETH_SPI_WRITE8(a)   spi1_exchangeByte(a)

typedef struct 
{
    uint8_t *start;
    uint8_t *currentLocation;
    uint16_t bufferLength;
    uint16_t dataLength;
 } memoryBuffer;
 
 typedef struct 
 {
     memoryBuffer txBuffer;
     memoryBuffer rxBuffer;
 } ksz_buffers;
 
 extern ksz_buffers dataBuffers;

 void TXBuffer_Init(void);
 void RXBuffer_Init(void);
 uint16_t BufferReceive(memoryBuffer *buffer, uint16_t length);
 void BufferReplaceAtOffset( memoryBuffer *buffer, char *data, uint16_t length, uint16_t offset);
 
 uint8_t BufferRead8(memoryBuffer *buffer);
 uint16_t BufferRead16(memoryBuffer *buffer);
 uint32_t BufferRead32(memoryBuffer *buffer);
 uint16_t BufferReadBlock(memoryBuffer *buffer,  void *data , uint16_t length);

 
 void BufferWrite8(memoryBuffer *buffer, uint8_t data);
 void BufferWrite16(memoryBuffer *buffer, uint16_t data);
 void BufferWrite32(memoryBuffer *buffer, uint32_t data);
 uint16_t BufferWriteString(memoryBuffer *buffer, const char *data);
 uint16_t BufferWriteBlock(memoryBuffer *buffer, char *data, uint16_t length);

 uint8_t BufferPeekCopy(memoryBuffer*, uint16_t);
 void BufferDump(memoryBuffer*, uint16_t );
 error_msg BufferRXtoTXCopy(memoryBuffer*, memoryBuffer*, uint16_t );
 void BufferReplacePacketLength(memoryBuffer *buffer, uint16_t data);

 void BufferSend(memoryBuffer *buffer);

 #endif