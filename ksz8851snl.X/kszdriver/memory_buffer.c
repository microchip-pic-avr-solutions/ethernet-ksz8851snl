/*
    \file   memory_buffer.c

    \brief  KSZ8851SNL ethernet controller buffer source file.

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
#include "memory_buffer.h"
#include <string.h>
#include "../mcc_generated_files/TCPIPLibrary/tcpip_types.h"
#include "../mcc_generated_files/TCPIPLibrary/ethernet_driver.h"
#include "../mcc_generated_files/TCPIPLibrary/network.h"

static uint8_t TXBuffer[TXBUFFER_SIZE];
static uint8_t RXBuffer[RXBUFFER_SIZE];
ksz_buffers dataBuffers;

/**
 * Initialize the TX memory buffer
 */
void TXBuffer_Init(void)
{
    memset(TXBuffer, 0, TXBUFFER_SIZE);
    dataBuffers.txBuffer.start = TXBuffer;
    dataBuffers.txBuffer.bufferLength = TXBUFFER_SIZE;
    dataBuffers.txBuffer.currentLocation = dataBuffers.txBuffer.start;
    dataBuffers.txBuffer.dataLength = 0;   
}

/**
 * Initialize the RX memory buffer
 */
void RXBuffer_Init(void)
{
    memset(RXBuffer, 0, RXBUFFER_SIZE);
    dataBuffers.rxBuffer.start = RXBuffer;
    dataBuffers.rxBuffer.bufferLength = RXBUFFER_SIZE;
    dataBuffers.rxBuffer.currentLocation = dataBuffers.rxBuffer.start;
    dataBuffers.rxBuffer.dataLength = 0;     
}
/**
 * Receive data from the hardware buffer
 * @param length
 */
uint16_t BufferReceive(memoryBuffer *buffer, uint16_t length)
{
    uint8_t *bufferEnd = buffer->start + buffer->bufferLength - 1;
    uint8_t *dataEnd = (buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start;
    uint16_t i;
     
     for(i=0; i<length; i++)
     {
         if (dataEnd > bufferEnd)
         {
             dataEnd = buffer->start;
         }
         *dataEnd = ETH_SPI_READ8();
         dataEnd++;
         buffer->dataLength++;
     }
    //Aligning the data frame to double word boundary
     for(length = length; length%4;length++)
     {
         *dataEnd = ETH_SPI_READ8();
         dataEnd++;
         buffer->dataLength++;
     }
     return length;
}

/**
 * Read 1 byte from memory buffer
 * @return
 */
 uint8_t BufferRead8(memoryBuffer *buffer)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint8_t readValue;
     
     readValue = *buffer->currentLocation;
     buffer->currentLocation++;
     buffer->dataLength--;
     if(buffer->currentLocation > bufferEnd)
     {
         buffer->currentLocation = buffer->start;
     }
         
     return readValue; 
 }
 
 /**
 * Read 2 byte from memory buffer
 * @return
 */
 uint16_t BufferRead16(memoryBuffer *buffer)
 {
     uint8_t *bufferEnd = buffer->start + buffer->bufferLength - 1;
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint8_t data[2];
     uint16_t readValue = 0;
     uint16_t i;
     
     for(i=0; i<(sizeof(data)) && buffer->dataLength > 0; i++)
     {         
         data[i] = *buffer->currentLocation;
         buffer->currentLocation++;
         buffer->dataLength--;
         if(buffer->currentLocation > bufferEnd)
         {
             buffer->currentLocation = buffer->start;
         }         
     }
     readValue = ((data[0] << 8) | (data[1]));
     
     return readValue; 
 }
 
 /**
 * Read 4 byte from memory buffer
 * @return
 */
 uint32_t BufferRead32(memoryBuffer *buffer)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint32_t readValue = 0;
     uint16_t i;
     uint8_t data[4];
     
     for(i=0; i<(sizeof(data)) && buffer->dataLength > 0; i++)
     {         
         data[i] = *buffer->currentLocation;
         buffer->currentLocation++;
         buffer->dataLength--;
         if(buffer->currentLocation > bufferEnd)
         {
             buffer->currentLocation = buffer->start;
         }         
     }
     readValue |= ((uint32_t)data[0] << 24)| ((uint32_t)data[1] << 16) | (data[2] << 8) | (data[3]);
     return readValue; 
 }
 
 /**
 * Read block of data from memory buffer
 * @return
 */
 uint16_t BufferReadBlock(memoryBuffer *buffer,void* data, uint16_t length)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint16_t i;
     char *p = data;
     
     for(i=0; i<length && buffer->dataLength > 0; i++)
     {         
         *p++ = *buffer->currentLocation;
         buffer->currentLocation++;
         buffer->dataLength--;
         if(buffer->currentLocation > bufferEnd)
         {
             buffer->currentLocation = buffer->start;
         }         
     }
     return i; 
 } 
 /**
 * Write 1 byte to memory buffer
 * @param data
 */
 void BufferWrite8(memoryBuffer *buffer, uint8_t data)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = (buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start;
     
     if (dataEnd > bufferEnd)
     {
         dataEnd = buffer->start;
     }
     *dataEnd = data;
     dataEnd++;
     buffer->dataLength++;
 }
 
 /**
 * Write 2 byte to memory buffer
 * @param data
 */
 void BufferWrite16(memoryBuffer *buffer, uint16_t data)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint16_t i;
     uint8_t value[2];
     value[0] = data >> 8;
     value[1] = data;
     
     for(i=0; i<2; i++)
     {
         if (dataEnd > bufferEnd)
         {
             dataEnd = buffer->start;
         }

         *dataEnd = value[i];
         dataEnd++;
         buffer->dataLength++;
     }
 }
 
 /**
 * Write 4 byte to memory buffer
 * @param data
 */
 void BufferWrite32(memoryBuffer *buffer, uint32_t data)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint16_t i;
     uint8_t value[4];
    
     value[0] = data >> 24;
     value[1] = data >>16;
     value[2] = data >> 8;
     value[3] = data;    
     
     for(i=0; (i<sizeof(value)); i++)
     {
         if (dataEnd > bufferEnd)
         {
             dataEnd = buffer->start;
         }
         *dataEnd = value[i];
         dataEnd++;
         buffer->dataLength++;
     }
 }

 /**
 * Write a string to memory buffer
 * @param data
 */
 uint16_t BufferWriteString(memoryBuffer *buffer, const char *data)
{
    uint16_t length;
    uint16_t i;    
    uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
    uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
    length = strlen(data);
    
    for(i=0; i<length; i++)
    {
        if (dataEnd > bufferEnd)
        {
            dataEnd = buffer->start;
        }
        *dataEnd = *data;
        dataEnd++;
        data++;
        buffer->dataLength++;
    }
    return length;
}
 
 /**
 * Write block of data to memory buffer
 * @param data
 * @param length
 */
 uint16_t BufferWriteBlock(memoryBuffer *buffer, char* data, uint16_t length)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint16_t i;
     
     for(i=0; i<length; i++)
     {
         if (dataEnd > bufferEnd)
         {
             dataEnd = buffer->start;
         }
         
         *dataEnd = *data;
         dataEnd++;
         data++;
         buffer->dataLength++;
     }
     return length;
 } 

/**
 * Replace the data at the specified offset
 * @param data
 * @param length
 * @param offset
 */
 void BufferReplaceAtOffset(memoryBuffer *buffer, char *data, uint16_t length, uint16_t offset)
 {
     uint16_t i;
     uint8_t *dataEnd;
     uint8_t *bufferEnd;
     
     offset = (offset + sizeof(controlWord)+ sizeof(packetLength));
     bufferEnd = (buffer->start + buffer->bufferLength - 1);
     dataEnd = (buffer->start + offset);

     for(i=0; ((i<length) && (i<buffer->dataLength)); i++)
     {
         *dataEnd = *data;
         dataEnd++;
         data++;
     }
 }
 
 /**
 * Update final packet length to memory buffer before transmitting
 * @param data
 */
 void BufferReplacePacketLength(memoryBuffer *buffer, uint16_t data)
 {
     uint16_t i=0;
     uint8_t value[2];
     uint8_t offset = 0;
     offset = offset + sizeof(controlWord);
    
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = (buffer->start + offset);
     value[0] = data;
     value[1] = data >> 8;
        
     for(i=0; ((i<2) && (i<buffer->dataLength)); i++)
     {
         if (dataEnd > bufferEnd)
         {
             dataEnd = buffer->start;
         }

         *dataEnd = value[i];
         dataEnd++;
     }
 }

 /**
 * Copy data from RX memory buffer to TX memory buffer
 * @param length
 * @return
 */
 error_msg BufferRXtoTXCopy(memoryBuffer *rxbuffer, memoryBuffer *txbuffer, uint16_t length)
 {
     uint8_t *ptr = rxbuffer ->currentLocation;
     uint8_t *txBufferEnd = txbuffer->start + txbuffer->bufferLength -1;
     uint8_t *rxBufferEnd = rxbuffer->start + rxbuffer->bufferLength -1;
     uint8_t *txDataEnd = ((txbuffer->currentLocation - txbuffer->start + txbuffer->dataLength) % txbuffer->bufferLength + txbuffer->start);
     uint16_t i;
     
     for(i=0; i<length; i++) 
     {
         if(txDataEnd > txBufferEnd)
         {
             txDataEnd = txbuffer->start;
         }
         
         *txDataEnd = *ptr;
         txDataEnd++;
         ptr++;
         txbuffer->dataLength++;
         
         if(ptr>rxBufferEnd)
         {
             ptr = rxbuffer->start;
         }
     }
     return SUCCESS;     
 }
 
 /**
 * Copy 1 byte from memory buffer
 * @param offset
 * @return
 */
 uint8_t BufferPeekCopy(memoryBuffer *buffer,uint16_t offset)
 {
     uint8_t *ptr = buffer->currentLocation;
     uint8_t *bufferEnd = buffer->start + buffer->bufferLength -1;
     uint8_t data;

     ptr = ptr + offset;
     data = *ptr;
     if(ptr > bufferEnd)
     {
         ptr = buffer->start;
     }
     return data;     
 }

 /**
 * Dump the specified length of data
 * @param length
 */
 void BufferDump(memoryBuffer *buffer, uint16_t length)
 {
     uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
     uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
     uint8_t data;
     uint16_t i;
     
     for(i=0; ((i<length) && (buffer->dataLength > 0)); i++)
     {
         data = *buffer->currentLocation;
         buffer->currentLocation++;
         buffer->dataLength--;
         if(buffer->currentLocation > bufferEnd)
         {
             buffer->currentLocation = buffer->start;
         }         
     }    
 }

 /**
 * Send data from memory buffer to hardware buffer for transmitting
 */
 void BufferSend(memoryBuffer *buffer)
 {
    uint8_t *bufferEnd = (buffer->start + buffer->bufferLength - 1);
    uint8_t *dataEnd = ((buffer->currentLocation - buffer->start + buffer->dataLength) % buffer->bufferLength + buffer->start);
    uint16_t initialLength;
    uint16_t I;
    packetLength = buffer->dataLength;

    BufferReplacePacketLength(&dataBuffers.txBuffer, packetLength);
    for(initialLength=0; buffer->dataLength > 0; initialLength++)
    {         
         ETH_SPI_WRITE8((*buffer->currentLocation));
         buffer->currentLocation++;
         buffer->dataLength--;
         if(buffer->currentLocation > bufferEnd)
         {
             buffer->currentLocation = buffer->start;
         }         
     }
    //Aligning the data frame to double word boundary
    for(I=initialLength; (I%4) ; I++)
    {
        ETH_SPI_WRITE8(0);  
    } 
 }
