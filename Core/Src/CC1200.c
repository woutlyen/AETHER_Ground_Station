/**
  ******************************************************************************
  * @file    CC1200.c
  * @author  Wout Lyen
  * @brief   This file contains the implementations of the CC1200 functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Aetherspace.
  * All rights reserved.
  *
  * This software is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "CC1200.h"
#include "CC1200_REG_DEF.h"
#include "cmsis_os2.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/

typedef enum {
    auto_NSS_release,
    manual_NSS_release
} NSS_Release_Mode;

typedef enum {
    StartOfPacket,
    EndOfPacket,
    Error
} Packet_Position;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef *hspi;
GPIO_TypeDef *CSPort;
uint32_t CSPin;

GPIO_TypeDef *UserMisoPort;
uint32_t UserMisoPin;

uint8_t current_state = 0;

/* Private function prototypes -----------------------------------------------*/

void CC1200_Reset(void);

void CC1200_ReadAllRegisters(void);

void CC1200_WriteRegister(uint16_t addr, uint8_t value);
void CC1200_WriteNormalRegister(uint16_t addr, uint8_t value);
void CC1200_WriteExtendedRegister(uint16_t addr, uint8_t value);

uint8_t CC1200_ReadNormalRegister(uint16_t addr);
uint8_t CC1200_ReadExtendedRegister(uint16_t addr);

uint8_t CC1200_GetTXFIFOSpace(void);


/* Private user code --------------------------------------------------------- */

void CC1200_SetSPIHandle(SPI_HandleTypeDef *hspi_handle, GPIO_TypeDef *cs_port, uint32_t cs_pin) {
    hspi = hspi_handle;
    CSPort = cs_port;
    CSPin = cs_pin;
}

void CC1200_SetUserMISOPins(GPIO_TypeDef *miso_port, uint32_t miso_pin) {
    UserMisoPort = miso_port;
    UserMisoPin = miso_pin;
}

void CC1200_Init(void) {
    // Reset the CC1200
    CC1200_Reset();

    // Initialize the CC1200 with the settings from CC1200_cfg
    for (uint16_t i = 0; i < CC1200_cfg_size; i++) {
        CC1200_WriteRegister(CC1200_cfg[i].addr, CC1200_cfg[i].value);
    }

    CC1200_ReadAllRegisters(); // Read back all registers to verify that they have been set correctly
}

void CC1200_CommandStrobe(uint8_t strobe) {
    if ((strobe > CC1200_SNOP) || (strobe < CC1200_SRES)) {
        // Invalid strobe command
        return;    
    }
    uint8_t tx_buf, rx_buf;
    tx_buf = strobe | CC1200_SINGLE_WRITE; // Strobe command

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);

    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(hspi, &tx_buf, &rx_buf, 1, HAL_MAX_DELAY);

    if (strobe != CC1200_SRES) {
        HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);
    }

    current_state = rx_buf; // & CC1200_STATUS_STATE_Msk; // Update current state based on status byte
}

uint8_t CC1200_GetState(void) {
    return current_state;
}

void CC1200_SplitAndTransmitPacket(uint8_t *data, uint8_t length){
    Packet_Position packet_position = StartOfPacket;
    uint8_t bytes_transmitted = 0;
    
    for (;;) {
        uint8_t chunk_size = (length - bytes_transmitted) > CC1200_TX_FIFO_SIZE ? CC1200_TX_FIFO_SIZE : (length - bytes_transmitted);
        if (packet_position == StartOfPacket) {
            CC1200_TransmitPacket(data, chunk_size);
            bytes_transmitted += chunk_size;
            packet_position = EndOfPacket; // After transmitting the first chunk, we will be at the end of the packet

            // Wait for flag from GPIO callback indicating that the packet has been transmitted over RF (Flag 0x00000002U)
            osThreadFlagsWait(0x00000002U, osFlagsWaitAny, osWaitForever);

        } else if (packet_position == EndOfPacket) {
            data[bytes_transmitted-2] = data[0];        // Add length byte at the beginning of the next chunk
            data[bytes_transmitted-1] = data[1] | 0x80; // Set the MSB bit for the next chunk, indicating EndOfPacket
            CC1200_TransmitPacket(data + bytes_transmitted-2, chunk_size+2);
            bytes_transmitted += chunk_size;

            if (bytes_transmitted < length) {
                // If we still have more data to transmit after this chunk, but we have already marked this chunk as EndOfPacket, 
                // then we have an error because we cannot transmit more than 2 chunks for a single packet.
                packet_position = Error;             
            }
            else {
                // We have transmitted all data, and the last chunk is correctly marked as EndOfPacket, so we can exit the loop
                break;
            }
        } else {
            // We have an error because we cannot transmit more than 2 chunks for a single packet.
            break;
        }

        // for(uint16_t i = 0; i < 4000; i++);

    }
}

void CC1200_TransmitPacket(uint8_t *data, uint8_t length) {

    CC1200_CommandStrobe(CC1200_SNOP);
    while (((current_state & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_TX) | 
    ((current_state & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_CALIBRATE) | 
    ((current_state & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_SETTLING) |
    ((current_state & CC1200_STATUS_RESERVED_Msk) != 0x0F)) {
        // Wait until we are no longer in the TX state before trying to transmit
        for(uint16_t i = 0; i < 1790; i++);
        CC1200_CommandStrobe(CC1200_SNOP);
    
    }

    if ((current_state & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_TXFIFO_ERR) {
        // If we get a TX FIFO error, we need to flush the TX FIFO before we can continue transmitting
        CC1200_CommandStrobe(CC1200_SIDLE);
        CC1200_CommandStrobe(CC1200_SFTX);
    }
    else {
        CC1200_CommandStrobe(CC1200_SIDLE); // Ensure we are in IDLE state before trying to transmit
    }

try_again:
    uint8_t amount_to_transmit = length; // The first byte of the data is the length byte
    uint8_t amount_transmitted = 0;

    while (amount_transmitted < amount_to_transmit) {
        uint8_t space = CC1200_GetTXFIFOSpace();
        if (amount_transmitted == 0){
            space--;
        }

        if (current_state == CC1200_STATUS_STATE_TXFIFO_ERR) {
            // If we get a TX FIFO error, we need to flush the TX FIFO before we can continue transmitting
            CC1200_CommandStrobe(CC1200_SIDLE);
            CC1200_CommandStrobe(CC1200_SFTX);
            goto try_again; // After flushing the TX FIFO, we can check the space again and try transmitting again
        }
        if (space > 0) {
            uint8_t diff = amount_to_transmit - amount_transmitted;
            uint8_t to_transmit = diff < space ? diff : space;

            // First send the burst write command with the TX FIFO address
            uint8_t cmd, ret;
            cmd = CC1200_BURST_WRITE | CC1200_TXFIFO; // Burst write command for TX FIFO

            HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);  // Reset NSS pin to start the transaction

            while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

            HAL_SPI_TransmitReceive(hspi, &cmd, &ret, 1, HAL_MAX_DELAY);

            if ((ret & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_TXFIFO_ERR) {
                // If we get a TX FIFO error, we need to flush the TX FIFO before we can continue transmitting
                HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS high to end the transaction

                CC1200_CommandStrobe(CC1200_SIDLE);
                CC1200_CommandStrobe(CC1200_SFTX);
                goto try_again; // After flushing the TX FIFO, we can check the space again and try transmitting again
            }

            // First send length byte
            if (amount_transmitted == 0){
                HAL_SPI_TransmitReceive(hspi, &amount_to_transmit, &ret, 1, HAL_MAX_DELAY);

                if ((ret & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_TXFIFO_ERR) {
                    // If we get a TX FIFO error, we need to flush the TX FIFO before we can continue transmitting
                    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS high to end the transaction

                    CC1200_CommandStrobe(CC1200_SIDLE);
                    CC1200_CommandStrobe(CC1200_SFTX);
                    goto try_again; // After flushing the TX FIFO, we can check the space again and try transmitting again
                }
            }

            // Then send the data to be transmitted
            uint8_t rx_buf[to_transmit];
            HAL_SPI_TransmitReceive_DMA(hspi, data + amount_transmitted, rx_buf, to_transmit);

            // Wait until HAL_SPI_TxCpltCallback send flag that the DMA transfer is completed    (Flag 0x00000001U)
            osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);

            HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS high to end the transaction

            amount_transmitted += to_transmit;
            current_state = rx_buf[to_transmit - 1] & CC1200_STATUS_STATE_Msk; // Update current state based on status byte

            for (uint8_t i = 0; i < to_transmit; i++) {
                if ((rx_buf[i] & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_TXFIFO_ERR) {
                    // If we get a TX FIFO error after transmitting data, we also need to flush the TX FIFO before we can continue transmitting
                    CC1200_CommandStrobe(CC1200_SIDLE);
                    CC1200_CommandStrobe(CC1200_SFTX);
                    goto try_again; // After flushing the TX FIFO, we can check the space again and try transmitting again
                }
            }
            
            if (amount_transmitted == amount_to_transmit) {
                CC1200_GetTXFIFOSpace();
                CC1200_CommandStrobe(CC1200_STX); // Strobe the TX command to start transmission of the packet in the TX FIFO
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
                // CC1200_CommandStrobe(CC1200_SNOP);
            }

        }
    }
}

void CC1200_ReceivePacket(uint8_t *buffer) {
    // First read the length byte to know how many bytes to read from the RX FIFO
    uint8_t cmd, ret;
    cmd = CC1200_BURST_READ | CC1200_RXFIFO; // Burst read command for RX FIFO
    HAL_SPI_TransmitReceive(hspi, &cmd, &ret, 1, HAL_MAX_DELAY);

    if ((ret & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_RXFIFO_ERR) {
        // If we get an RX FIFO error, we need to flush the RX FIFO before we can continue receiving
        CC1200_CommandStrobe(CC1200_SFRX);
        return; // After flushing the RX FIFO, we can return and wait for the next packet to be received
    }

    uint8_t length;
    HAL_SPI_TransmitReceive(hspi, &cmd, &length, 1, HAL_MAX_DELAY); // Read the length byte

    if (length > 0) {
        uint8_t ret[length];
        HAL_SPI_TransmitReceive(hspi, &cmd, ret, length, HAL_MAX_DELAY); // Read the rest of the packet based on the length byte
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = ret[i]; // Copy the received data into the provided buffer
        }
    }
}

/* Private Functions */

void CC1200_Reset(void) {
    // To reset the CC1200, we can strobe the SRES command
    CC1200_CommandStrobe(CC1200_SRES); // We will manually control the NSS pin for the reset sequence

    // Wait until MISO pin goes low, which indicates that the CC1200 is ready to receive commands after reset
    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    // Set NSS pin high to end the reset sequence
    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);
}

void CC1200_ReadAllRegisters(void) {
    // This function can be used for debugging purposes to read all the registers of the CC1200 and print their values
    for (uint16_t addr = 0; addr <= CC1200_MAX_NORM_ADDR; addr++) {
        CC1200_ReadNormalRegister(addr); // Read normal registers
    }
    for (uint16_t addr = 0; addr <= 0xFF; addr++) {
        CC1200_ReadExtendedRegister(addr); // Read extended registers
    }
}

void CC1200_WriteRegister(uint16_t addr, uint8_t value) {
    if ((addr >> 8) == CC1200_EXTENDED_ADDR) {
        // Extended register
        CC1200_WriteExtendedRegister(addr, value);
    } else if ((addr >> 8) == 0) {
        // Standard register
        CC1200_WriteNormalRegister(addr, value);
    }
}

void CC1200_WriteNormalRegister(uint16_t addr, uint8_t value) {
    if ((addr & 0xFF) > CC1200_MAX_NORM_ADDR) {
        // Invalid normal register address
        return;    
    }
    uint8_t tx_buf[2], rx_buf[2];
    tx_buf[0] = addr | CC1200_SINGLE_WRITE; // Address with write flag
    tx_buf[1] = value;                      // Value to write

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);   // Reset NSS pin to start the transaction

    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);     // Set NSS high after the transaction

    current_state = rx_buf[1] & CC1200_STATUS_STATE_Msk; // Update current state based on status byte
}

void CC1200_WriteExtendedRegister(uint16_t addr, uint8_t value) {
    uint8_t tx_buf[3], rx_buf[3];
    tx_buf[0] = CC1200_EXTENDED_ADDR | CC1200_SINGLE_WRITE; // Extended address flag with write flag
    tx_buf[1] = addr & 0xFF;                                // Extended register address
    tx_buf[2] = value;                                      // Value to write

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);   // Reset NSS pin to start the transaction

    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 3, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);     // Set NSS high after the transaction

    current_state = rx_buf[2] & CC1200_STATUS_STATE_Msk;    // Update current state based on status byte
}

uint8_t CC1200_ReadNormalRegister(uint16_t addr) {
    if ((addr & 0xFF) > CC1200_MAX_NORM_ADDR) {
        // Invalid normal register address
        return 0;    
    }
    uint8_t tx_buf[2], rx_buf[2];
    tx_buf[0] = addr | CC1200_SINGLE_READ; // Address with read flag
    tx_buf[1] = 0;                         // Dummy byte for reading

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);   // Reset NSS pin to start the transaction

    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);     // Set NSS high after the transaction

    current_state = rx_buf[0] & CC1200_STATUS_STATE_Msk; // Update current state based on status byte
    return rx_buf[1]; // The value of the register is returned in the last byte of the response
}

uint8_t CC1200_ReadExtendedRegister(uint16_t addr) {
    uint8_t tx_buf[3], rx_buf[3];
    tx_buf[0] = CC1200_EXTENDED_ADDR | CC1200_SINGLE_READ; // Extended address flag with read flag
    tx_buf[1] = addr & 0xFF;                               // Extended register address
    tx_buf[2] = 0;                                         // Dummy byte for reading

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);   // Reset NSS pin to start the transaction

    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 3, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);     // Set NSS high after the transaction

    current_state = rx_buf[1] & CC1200_STATUS_STATE_Msk;   // Update current state based on status byte

    return rx_buf[2]; // The value of the extended register is returned in the last byte of the response
}

uint8_t CC1200_GetTXFIFOSpace(void) {
    // The number of bytes in the TX FIFO is given by the NUM_TXBYTES extended register, and the FIFO size is 128 bytes, so the space available is 128 minus the number of bytes currently in the FIFO
    uint8_t num_tx_bytes = CC1200_ReadExtendedRegister(CC1200_NUM_TXBYTES); // NUM_TXBYTES register address
    if (num_tx_bytes > 128) {
        // This should never happen, but if it does, we can just return 0 to indicate that there is no space available in the TX FIFO
        return 0;
    }
    uint8_t space = 128 - num_tx_bytes;
    return space;
}