/**
 * @file        spitest.c
 * @brief       Busmust X8PI board control over SPI from Raspberry Pi 4B.
 * @author      busmust
 * @version     1.0.0.1
 * @copyright   Copyright 2022 by Busmust Tech Co.,Ltd <br>
 *              All rights reserved. Property of Busmust Tech Co.,Ltd.<br>
 *              Restricted rights to use, duplicate or disclose of this code are granted through contract.
 */
#include <fcntl.h>				//Needed for SPI port
#include <sys/ioctl.h>			//Needed for SPI port
#include <linux/spi/spidev.h>	//Needed for SPI port
#include <unistd.h>			    //Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>


 ///////////////////////////////////////////////////////////////////////// PI SPI Driver /////////////////////////////////////////////////////////////////////////

int spi_dev_id = 6;
int spi_cs_id = 0;
int spi_cs_fd;				//file descriptor for the SPI device
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;

//spi_device	0=CS0, 1=CS1
int DRV_SPI_Open()
{
    int status_value = -1;


    //----- SET SPI MODE -----
    //SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    //SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    spi_mode = SPI_MODE_0;

    //----- SET BITS PER WORD -----
    spi_bitsPerWord = 8;

    //----- SET SPI BUS SPEED -----
    spi_speed = 1000000;		//1000000 = 1MHz (1uS per bit) 

    char dev[] = "/dev/spidev0.0";
    dev[11] = '0' + spi_dev_id;
    dev[13] = '0' + spi_cs_id;
    printf("Open device: %s\n", dev);
    spi_cs_fd = open(dev, O_RDWR);
    if (spi_cs_fd < 0)
    {
        perror("Error - Could not open SPI device");
        exit(1);
    }

    status_value = ioctl(spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (status_value < 0)
    {
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (status_value < 0)
    {
        perror("Could not set SPIMode (RD)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
    if (status_value < 0)
    {
        perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
    if (status_value < 0)
    {
        perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (status_value < 0)
    {
        perror("Could not set SPI speed (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (status_value < 0)
    {
        perror("Could not set SPI speed (RD)...ioctl fail");
        exit(1);
    }
    return(status_value);
}

int DRV_SPI_Close()
{
    int status_value = -1;

    status_value = close(spi_cs_fd);
    if (status_value < 0)
    {
        perror("Error - Could not close SPI device");
        exit(1);
    }
    return(status_value);
}

int DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t* SpiTxData, uint8_t* SpiRxData, uint16_t spiTransferSize)
{
    struct spi_ioc_transfer spi;
    int retVal = -1;

    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long)SpiTxData;    //transmit from "data"
    spi.rx_buf = (unsigned long)SpiRxData;    //receive into "data"
    spi.len = spiTransferSize;
    spi.delay_usecs = 0;
    spi.speed_hz = spi_speed;
    spi.bits_per_word = spi_bitsPerWord;
    spi.cs_change = 0;						//0=Set CS high after a transfer, 1=leave CS set low

    retVal = ioctl(spi_cs_fd, SPI_IOC_MESSAGE(1), &spi);
    //printf("ioctl(%p, %p, %d):%d\n", SpiTxData, SpiRxData, spiTransferSize, retVal);
    if (retVal < 0)
    {
        perror("Error - Problem transmitting spi data..ioctl");
        exit(1);
    }

    return (retVal == spiTransferSize ? 0 : -1);
}


///////////////////////////////////////////////////////////////////////// MCP2518 Driver /////////////////////////////////////////////////////////////////////////
// *****************************************************************************
// *****************************************************************************
/* SPI Instruction Set */

#define cINSTRUCTION_RESET			0x00
#define cINSTRUCTION_READ			0x03
#define cINSTRUCTION_READ_CRC       0x0B
#define cINSTRUCTION_WRITE			0x02
#define cINSTRUCTION_WRITE_CRC      0x0A
#define cINSTRUCTION_WRITE_SAFE     0x0C

#define cRAMADDR_START  0x400

// *****************************************************************************
// *****************************************************************************
// Section: Defines
#define CRCBASE    0xFFFF
#define CRCUPPER   1
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// *****************************************************************************
// *****************************************************************************
// Section: Variables

//! SPI Transmit buffer
uint8_t spiTransmitBuffer[511]; /* +2 to include command and address */

//! SPI Receive buffer
uint8_t spiReceiveBuffer[511]; /* +2 to include command and address */

typedef uint8_t CANFDSPI_MODULE_ID;

//! Reverse order of bits in byte
const uint8_t BitReverseTable256[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

//! Look-up table for CRC calculation
const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


// *****************************************************************************
// *****************************************************************************
// Section: Reset

int8_t DRV_CANFDSPI_Reset(CANFDSPI_MODULE_ID index)
{
    uint16_t spiTransferSize = 2;
    int8_t spiTransferError = 0;

    // Compose command
    spiTransmitBuffer[0] = (uint8_t)(cINSTRUCTION_RESET << 4);
    spiTransmitBuffer[1] = 0;

    spiTransferError = DRV_SPI_TransferData(index, spiTransmitBuffer, NULL, spiTransferSize);

    return spiTransferError;
}

uint16_t DRV_CANFDSPI_CalculateCRC16(uint8_t* data, uint16_t size)
{
    uint16_t init = CRCBASE;

    while (size--)
    {
        register uint8_t index = ((uint8_t)(init >> 8)) ^ *data++;
        init = (init << 8) ^ crc16_table[index];
    }

    return init;
}

int8_t DRV_CANFDSPI_ReadByteArrayWithCRC(CANFDSPI_MODULE_ID index, uint16_t address,
    uint8_t* rxd, uint16_t nBytes, bool fromRam, bool* crcIsCorrect)
{
    uint8_t i;
    uint16_t crcFromSpiSlave = 0;
    uint16_t crcAtController = 0;
    uint16_t spiTransferSize = nBytes + 5; //first two bytes for sending command & address, third for size, last two bytes for CRC
    int8_t spiTransferError = 0;

    // Compose command
    spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ_CRC << 4) + ((address >> 8) & 0xF));
    spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
    if (fromRam) {
        spiTransmitBuffer[2] = (uint8_t)(nBytes >> 2);
    }
    else {
        spiTransmitBuffer[2] = (uint8_t)(nBytes);
    }

    // Clear data
    for (i = 3; i < spiTransferSize; i++) {
        spiTransmitBuffer[i] = 0;
    }

    spiTransferError = DRV_SPI_TransferData(index, spiTransmitBuffer, spiReceiveBuffer, spiTransferSize);
    if (spiTransferError) {
        return spiTransferError;
    }

    // Get CRC from controller
    crcFromSpiSlave = (uint16_t)(spiReceiveBuffer[spiTransferSize - 2] << 8) + (uint16_t)(spiReceiveBuffer[spiTransferSize - 1]);

    // Use the receive buffer to calculate CRC
    // First three bytes need to be command
    spiReceiveBuffer[0] = spiTransmitBuffer[0];
    spiReceiveBuffer[1] = spiTransmitBuffer[1];
    spiReceiveBuffer[2] = spiTransmitBuffer[2];
    crcAtController = DRV_CANFDSPI_CalculateCRC16(spiReceiveBuffer, nBytes + 3);

    // Compare CRC readings
    if (crcFromSpiSlave == crcAtController) {
        *crcIsCorrect = true;
    }
    else {
        *crcIsCorrect = false;
    }

    // Update data
    for (i = 0; i < nBytes; i++) {
        rxd[i] = spiReceiveBuffer[i + 3];
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_RamTest(CANFDSPI_MODULE_ID index, uint32_t maxErrorCount)
{
    int8_t spiTransferError = 0;
    uint16_t a = cRAMADDR_START;
    uint32_t k;
    uint32_t crcErrorCount = 0;

    /* RAM TEST with different burst length */
    for (k = 4; k <= 76; k += 4)
    {
        uint16_t i;
        a = cRAMADDR_START + k * 4;
        spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((a >> 8) & 0xF));
        spiTransmitBuffer[1] = (uint8_t)(a & 0xFF);
        for (i = 0; i < k; i++) {
            spiTransmitBuffer[i + 2] = ((k + i) & 0xFF);
            spiReceiveBuffer[i + 2] = ~((k + i) & 0xFF);
        }

        // Write test pattern to RAM
        spiTransferError = DRV_SPI_TransferData(index, spiTransmitBuffer, NULL, k + 2);
        if (spiTransferError) {
            return -2;
        }

        bool crcIsCorrect = false;
        int offset = 0;
        // Read test pattern from RAM with CRC protection
        while ((DRV_CANFDSPI_ReadByteArrayWithCRC(index, a, spiReceiveBuffer, k, true, &crcIsCorrect) != 0 || !crcIsCorrect) && crcErrorCount < maxErrorCount)
        {
            crcErrorCount++;
        }
        if (crcErrorCount > 0)
        {
            return crcErrorCount;
        }

        // Verify read data against written data
        for (i = 0; i < k; i++) {
            if (spiReceiveBuffer[i + offset] != ((k + i) & 0xFF))
            {
                spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((a >> 8) & 0xF));
                spiTransmitBuffer[1] = (uint8_t)(a & 0xFF);
                spiTransferError = DRV_SPI_TransferData(index, spiTransmitBuffer, spiReceiveBuffer, k + 2);
                if (spiTransferError) {
                    return -5;
                }
                return -4;
            }
        }
    }

    return spiTransferError;
}


///////////////////////////////////////////////////////////////////////// Main Entry /////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    int error_code;
    int num_of_test;

    printf("==== BUSMUST X8PI SPI TEST ========\n");
    spi_dev_id = argc > 1 ? strtol(argv[1], 0, 10) : 4;
    spi_cs_id = argc > 2 ? strtol(argv[2], 0, 10) : 0;
    num_of_test = argc > 3 ? strtol(argv[3], 0, 10) : -1;
    DRV_SPI_Open();

    // Reset device
    error_code = DRV_CANFDSPI_Reset(0);
    if (error_code)
    {
        printf("Failed to reset MCP2518FD, error=%d.\n", error_code);
        return error_code;
    }

    int i;
    int fail_count = 0;
    int total_count = 0;
    for (i = 0; i < num_of_test || num_of_test < 0; i++)
    {
        printf("Testing MCP2518FD[SPI=%d, CS=%d] %d/%d ...\n", spi_dev_id, spi_cs_id, total_count - fail_count, total_count);
        error_code = DRV_CANFDSPI_RamTest(0, 3);
        total_count++;
        if (error_code)
        {
            fail_count++;
            printf("Failed to init MCP2518FD internal RAM, error=%d\n", error_code);
        }
    }

    DRV_SPI_Close();
    return fail_count;
}
/**
 * End of file
 */
