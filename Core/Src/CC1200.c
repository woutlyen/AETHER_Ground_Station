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

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef *hspi;
GPIO_TypeDef *CSPort;
uint32_t CSPin;

GPIO_TypeDef *UserMisoPort;
uint32_t UserMisoPin;

uint8_t current_state = 0;

uint8_t emptyBuffer[128] = {0}; // Buffer filled with zeros that can be used for transmitting empty packets when needed

/* Private function prototypes -----------------------------------------------*/

void CC1200_Reset(void);

void CC1200_ReadAllRegisters(void);

void CC1200_WriteRegister(uint16_t addr, uint8_t value);
void CC1200_WriteNormalRegister(uint16_t addr, uint8_t value);
void CC1200_WriteExtendedRegister(uint16_t addr, uint8_t value);

uint8_t CC1200_ReadNormalRegister(uint16_t addr);
uint8_t CC1200_ReadExtendedRegister(uint16_t addr);

uint8_t CC1200_GetTXFIFOSpace(void);
uint8_t CC1200_GetRXFIFOLength(void);


/* Private user code --------------------------------------------------------- */

/**
  * @brief  Sets the SPI handle and GPIO pins for the CC1200
  * @param  hspi_handle: Pointer to the SPI handle
  * @param  cs_port: Pointer to the GPIO port for the chip select pin
  * @param  cs_pin: The chip select pin
  * @retval None
  */
void CC1200_SetSPIHandle(SPI_HandleTypeDef *hspi_handle, GPIO_TypeDef *cs_port, uint32_t cs_pin) {
    hspi = hspi_handle;
    CSPort = cs_port;
    CSPin = cs_pin;
}

/**
  * @brief  Sets the GPIO pins for the User MISO line
  * @param  miso_port: Pointer to the GPIO port for the MISO pin
  * @param  miso_pin: The MISO pin
  * @retval None
  */
void CC1200_SetUserMISOPins(GPIO_TypeDef *miso_port, uint32_t miso_pin) {
    UserMisoPort = miso_port;
    UserMisoPin = miso_pin;
}

/**
  * @brief  Initializes the CC1200
  * @retval None
  */
void CC1200_Init(void) {
    // Reset the CC1200
    CC1200_Reset();

    // Initialize the CC1200 with the settings from CC1200_cfg
    for (uint16_t i = 0; i < CC1200_cfg_size; i++) {
        CC1200_WriteRegister(CC1200_cfg[i].addr, CC1200_cfg[i].value);
    }

    CC1200_ReadAllRegisters(); // Read back all registers to verify that they have been set correctly
}

/**
  * @brief  Sends a command strobe to the CC1200
  * @param  strobe: The strobe command to send
  * @retval None
  */
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

/**
  * @brief  Gets the current state of the CC1200
  * @retval The current state
  */
uint8_t CC1200_GetState(void) {
    return current_state;
}

/**
  * @brief  Transmits a packet using the CC1200
  * @param  data: Pointer to the data to transmit
  * @param  length: The length of the data to transmit
  * @retval None
  */
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
        if ((amount_transmitted == 0) & (space > 0)) {
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

            if (amount_transmitted == 0) {
                CC1200_CommandStrobe(CC1200_STX); // Strobe the TX command to start transmission of the packet in the TX FIFO
            }

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
            
            if (amount_transmitted < amount_to_transmit) {
                // Wait for flag from GPIO callback indicating that the RF module is ready to transmit the next part of the packet if needed (Flag 0x00000004U)
                osThreadFlagsWait(0x00000004U, osFlagsWaitAny, osWaitForever);
            }

        } else {
            // Wait for flag from GPIO callback indicating that the RF module is ready to transmit the next part of the packet if needed (Flag 0x00000004U)
            osThreadFlagsWait(0x00000004U, osFlagsWaitAny, osWaitForever);
        }
    }
}

/** @brief Receive the header of a packet from the CC1200 RX FIFO
  * @param buffer Pointer to the buffer where the header will be stored
  * @retval None
  */
void CC1200_ReceiveHeader(uint8_t *buffer) {
 
retry_RX:

    // Wait until we receive a flag from the GPIO callback indicating that the PKT_SYNC_RXTX pin has changed state to HIGH, which indicates that the header of the packet in the CC1200 RXFIFO is ready to be read (Flag 0x00000040U)
    osThreadFlagsClear(0x00000020U | 0x00000040U | 0x00000080U); // Clear flags in case they were set from a previous packet reception
    osThreadFlagsWait(0x00000040U, osFlagsWaitAny, osWaitForever);

    for(uint8_t i = 0; i < 10; i++) {
        if (i == 9) {
            // If after waiting for a while we still don't have at least 3 bytes in the RX FIFO, then we can assume that something went wrong and we can discard this packet and wait for the next one  
            // Put CC1200 in RX Mode
            CC1200_CommandStrobe(CC1200_SIDLE);
            CC1200_CommandStrobe(CC1200_SFRX);
            CC1200_CommandStrobe(CC1200_SRX);
            goto retry_RX; // We can goto begin of function and wait for the next packet to be received
         } else if (CC1200_GetRXFIFOLength() >= 3) {
                break; // We have at least 3 bytes in the RX FIFO, which means we can read the header and length bytes of the packet, so we can break out of this loop and continue with processing this packet
        }
    }

    // Receive header (Variable Length of CC1200 packet + Length of camera/sensor packet (should be the same) + Source ID) of the packet in the CC1200 RXFIFO
    uint8_t cmd, ret;
    cmd = CC1200_BURST_READ | CC1200_RXFIFO; // Burst read command for RX FIFO

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);  // Reset NSS pin to start the transaction

    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    HAL_SPI_TransmitReceive(hspi, &cmd, &ret, 1, HAL_MAX_DELAY);

    if ((ret & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_RXFIFO_ERR) {
        HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS high to end the transaction
        // Put CC1200 in RX Mode
        CC1200_CommandStrobe(CC1200_SIDLE);
        CC1200_CommandStrobe(CC1200_SFRX);
        CC1200_CommandStrobe(CC1200_SRX);
        goto retry_RX; // We can goto begin of function and wait for the next packet to be received
    }

    HAL_SPI_TransmitReceive(hspi, &emptyBuffer[0], buffer, 3, HAL_MAX_DELAY); // Read the header bytes

    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS pin to start the transaction

    if ((buffer[0] < 6) | (buffer[0] > 254) | (buffer[1] < 6) | (buffer[1] > 254) | (buffer[0] != buffer[1])) { // | (buffer[2] > 0x02)) {
        // If the length bytes in the header are not valid, or if the source ID is not valid, then we can discard this packet and wait for the next one
        // Put CC1200 in RX Mode
        CC1200_CommandStrobe(CC1200_SIDLE);
        CC1200_CommandStrobe(CC1200_SFRX);
        CC1200_CommandStrobe(CC1200_SRX);
        goto retry_RX; // We can goto begin of function and wait for the next packet to be received
    }
}


/** @brief Receive the payload of a packet from the CC1200 RX FIFO
  * @param buffer Pointer to the buffer where the payload will be stored
  * @param length Length of the payload to receive
  * @retval None
  */
uint8_t CC1200_ReceivePayload(uint8_t *buffer, uint8_t length) {
    
    uint8_t bytes_received = 0;

    while (bytes_received < length) {

        // Wait until we receive a flag from the GPIO callback indicating that the RX FIFO threshold has been reached or the end of the packet is ready to be read from the CC1200 RXFIFO (Flag 0x00000020U or 0x00000080U)
        if (length - bytes_received > 10) { //TODO: rand geval
            uint32_t flags = osThreadFlagsWait(0x00000020U | 0x00000080U, osFlagsWaitAny, osWaitForever);
            if ((flags & 0x00000080U) == 0x00000080U) {
                // Put CC1200 in RX Mode
                CC1200_CommandStrobe(CC1200_SIDLE);
                CC1200_CommandStrobe(CC1200_SRX);
            }
        }

        // First get the number of bytes available in the RX FIFO
        uint8_t num_rx_bytes = CC1200_GetRXFIFOLength();

        if (num_rx_bytes != 0) {
            uint8_t cmd, ret;
            cmd = CC1200_BURST_READ | CC1200_RXFIFO; // Burst read command for RX FIFO

            HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_RESET);  // Reset NSS pin to start the transaction

            while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

            HAL_SPI_TransmitReceive(hspi, &cmd, &ret, 1, HAL_MAX_DELAY);

            if ((ret & CC1200_STATUS_STATE_Msk) == CC1200_STATUS_STATE_RXFIFO_ERR) {
                HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS high to end the transaction

                // Put CC1200 in RX Mode
                CC1200_CommandStrobe(CC1200_SIDLE);
                CC1200_CommandStrobe(CC1200_SFRX);
                CC1200_CommandStrobe(CC1200_SRX);

                return 1; // We can return 1 to indicate that we failed to receive the payload
            }

            // uint8_t to_read = num_rx_bytes < (length - bytes_received) ? num_rx_bytes : (length - bytes_received);
            // uint8_t rx_buf[to_read];
            // HAL_SPI_TransmitReceive_DMA(hspi, &emptyBuffer[0], buffer, to_read);

            HAL_SPI_TransmitReceive_DMA(hspi, &emptyBuffer[0], buffer + bytes_received, num_rx_bytes);

            // Wait until HAL_SPI_TxRxCpltCallback send flag that the DMA transfer is completed    (Flag 0x00000010U)
            osThreadFlagsWait(0x00000010U, osFlagsWaitAny, osWaitForever);

            HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);  // Set NSS high to end the transaction

            bytes_received += num_rx_bytes;
        }
    }

    // Check for CRC bit in the last byte of the payload, which is set by the CC1200 if the CRC check of the received packet is correct
    if ((buffer[length - 1] & 0x80) != 0x80) {
        // Put CC1200 in RX Mode
        CC1200_CommandStrobe(CC1200_SIDLE);
        CC1200_CommandStrobe(CC1200_SFRX);
        CC1200_CommandStrobe(CC1200_SRX);
        
        // If the CRC bit is not set, then we can discard this packet and return an error
        return 2; // We can return 2 to indicate that we failed to receive the payload due to a CRC error
    }

    return 0; // We can return 0 to indicate that we successfully received the full payload
}

/* Private Functions */

/**
 * @brief  Resets the CC1200
 * @retval None
 */
void CC1200_Reset(void) {
    // To reset the CC1200, we can strobe the SRES command
    CC1200_CommandStrobe(CC1200_SRES); // We will manually control the NSS pin for the reset sequence

    // Wait until MISO pin goes low, which indicates that the CC1200 is ready to receive commands after reset
    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(UserMisoPort, UserMisoPin) == GPIO_PIN_SET);

    // Set NSS pin high to end the reset sequence
    HAL_GPIO_WritePin(CSPort, CSPin, GPIO_PIN_SET);
}

/**
 * @brief  Reads all registers of the CC1200
 * @retval None
 */
void CC1200_ReadAllRegisters(void) {
    // This function can be used for debugging purposes to read all the registers of the CC1200 and print their values
    for (uint16_t addr = 0; addr <= CC1200_MAX_NORM_ADDR; addr++) {
        CC1200_ReadNormalRegister(addr); // Read normal registers
    }
    for (uint16_t addr = 0; addr <= 0xFF; addr++) {
        CC1200_ReadExtendedRegister(addr); // Read extended registers
    }
}

/**
 * @brief  Writes a value to a register in the CC1200
 * @param  addr: The address of the register to write to
 * @param  value: The value to write
 * @retval None
 */
void CC1200_WriteRegister(uint16_t addr, uint8_t value) {
    if ((addr >> 8) == CC1200_EXTENDED_ADDR) {
        // Extended register
        CC1200_WriteExtendedRegister(addr, value);
    } else if ((addr >> 8) == 0) {
        // Standard register
        CC1200_WriteNormalRegister(addr, value);
    }
}

/**
 * @brief  Writes a value to a normal register in the CC1200
 * @param  addr: The address of the normal register to write to
 * @param  value: The value to write
 * @retval None
 */
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

/**
 * @brief  Writes a value to an extended register in the CC1200
 * @param  addr: The address of the extended register to write to
 * @param  value: The value to write
 * @retval None
 */
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

/**
 * @brief  Reads a normal register in the CC1200
 * @param  addr: The address of the normal register to read
 * @retval The value of the register
 */
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

/**
 * @brief  Reads an extended register in the CC1200
 * @param  addr: The address of the extended register to read
 * @retval The value of the register
 */
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

/**
 * @brief  Gets the available space in the TX FIFO
 * @retval The number of bytes available in the TX FIFO
 */
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

/**
 * @brief  Gets the number of bytes in the RX FIFO
 * @retval The number of bytes in the RX FIFO
 */
uint8_t CC1200_GetRXFIFOLength(void) {
    // The number of bytes in the RX FIFO is given by the NUM_RXBYTES extended register
    uint8_t num_rx_bytes = CC1200_ReadExtendedRegister(CC1200_NUM_RXBYTES); // NUM_RXBYTES register address
    return num_rx_bytes;
}