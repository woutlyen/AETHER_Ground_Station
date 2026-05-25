/**
  ******************************************************************************
  * @file    CC1200.h
  * @author  Wout Lyen
  * @brief   This file contains the headers of the CC1200 functions.
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

#ifndef CC1200_H
#define CC1200_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"

#include "CC1200_REG.h"
#include "CC1200_REG_DEF.h"

/* SPI flags */
#define CC1200_READ_Pos         (7U)
#define CC1200_READ_Msk         (0x1 << CC1200_READ_Pos)                    /*!< 0b10000000     */
#define CC1200_READ             CC1200_READ_Msk                             /*!< CC1200 Read    */

#define CC1200_BURST_Pos        (6U)
#define CC1200_BURST_Msk        (0x1 << CC1200_BURST_Pos)                   /*!< 0b01000000     */
#define CC1200_BURST            CC1200_BURST_Msk                            /*!< CC1200 Burst   */

#define CC1200_SINGLE_WRITE     0x00
#define CC1200_BURST_WRITE      CC1200_BURST
#define CC1200_SINGLE_READ      CC1200_READ
#define CC1200_BURST_READ       (CC1200_READ | CC1200_BURST)
#define CC1200_EXTENDED_ADDR    0x2F
#define CC1200_MAX_NORM_ADDR    0x2E

/* TX/RX FIFO */
#define CC1200_FIFO       0x3F
#define CC1200_TXFIFO     CC1200_FIFO
#define CC1200_RXFIFO     CC1200_FIFO

#define CC1200_TX_FIFO_SIZE 125

/* Chip Status Byte */
#define CC1200_STATUS_CHIP_RDY_Pos    (7U)
#define CC1200_STATUS_CHIP_RDY_Msk    (0x1 << CC1200_STATUS_CHIP_RDY_Pos)   /*!< 0b10000000   */
#define CC1200_STATUS_CHIP_RDY        CC1200_STATUS_CHIP_RDY_Msk            /*!< CC1200 Chip Ready */

#define CC1200_STATUS_STATE_Pos         (4U)
#define CC1200_STATUS_STATE_IDLE        (0x0 << CC1200_STATUS_STATE_Pos)    /*!< 0b00000000   IDLE state        */
#define CC1200_STATUS_STATE_RX          (0x1 << CC1200_STATUS_STATE_Pos)    /*!< 0b00010000   Receive state     */
#define CC1200_STATUS_STATE_TX          (0x2 << CC1200_STATUS_STATE_Pos)    /*!< 0b00100000   Transmit state    */
#define CC1200_STATUS_STATE_FSTXON      (0x3 << CC1200_STATUS_STATE_Pos)    /*!< 0b00110000   Fast TX ready     */
#define CC1200_STATUS_STATE_CALIBRATE   (0x4 << CC1200_STATUS_STATE_Pos)    /*!< 0b01000000   Frequency synthesizer calibration is running */
#define CC1200_STATUS_STATE_SETTLING    (0x5 << CC1200_STATUS_STATE_Pos)    /*!< 0b01010000   PLL is settling   */
#define CC1200_STATUS_STATE_RXFIFO_ERR  (0x6 << CC1200_STATUS_STATE_Pos)    /*!< 0b01100000   RX FIFO has over/underflowed. Read out any useful data, then flush the FIFO with an SFRX strobe*/
#define CC1200_STATUS_STATE_TXFIFO_ERR  (0x7 << CC1200_STATUS_STATE_Pos)    /*!< 0b01110000   TX FIFO has over/underflowed. Flush the FIFO with an SFTX strobe*/
#define CC1200_STATUS_STATE_Msk         (0x7 << CC1200_STATUS_STATE_Pos)    /*!< 0b01110000   */

#define CC1200_STATUS_RESERVED_Msk       (0xF << 0)                         /*!< 0b00001111   Reserved bits in the status byte */

/* Command strobes */
#define CC1200_SRES             0x30    // Reset chip
#define CC1200_SFSTXON          0x31    // Enable and calibrate frequency synthesizer (if SETTLING_CFG.FS_AUTOCAL = 1). If in RX and PKT_CFG2.CCA_MODE ≠ 0: Go to a wait state where only the synthesizer is running (for quick RX/TX turnaround). 
#define CC1200_SXOFF            0x32    // Enter XOFF state when CSn is de-asserted
#define CC1200_SCAL             0x33    // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (SETTLING_CFG.FS_AUTOCAL = 0)
#define CC1200_SRX              0x34    // Enable RX. Perform calibration first if coming from IDLE and SETTLING_CFG.FS_AUTOCAL = 1
#define CC1200_STX              0x35    // In IDLE state: Enable TX. Perform calibration first if SETTLING_CFG.FS_AUTOCAL = 1. If in RX state and PKT_CFG2.CCA_MODE ≠ 0: Only go to TX if channel is clear
#define CC1200_SIDLE            0x36    // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1200_SAFC             0x37    // Automatic Frequency Compensation
#define CC1200_SWOR             0x38    // Start automatic RX polling sequence (Wake-On-Radio) as described in Section 9.6 if WOR_CFG0.RC_PD = 0 
#define CC1200_SPWD             0x39    // Enter SLEEP mode when CSn is de-asserted
#define CC1200_SFRX             0x3A    // Flush the RX FIFO. Only issue SFRX in IDLE or RX_FIFO_ERR states
#define CC1200_SFTX             0x3B    // Flush the TX FIFO. Only issue SFTX in IDLE or TX_FIFO_ERR states
#define CC1200_SWORRST          0x3C    // Reset the eWOR timer to the Event1 value
#define CC1200_SNOP             0x3D    // No operation. May be used to get access to the chip status byte

/* Function prototypes */
void CC1200_SetSPIHandle(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CSPort, uint32_t CSPin);
void CC1200_SetUserMISOPins(GPIO_TypeDef *miso_port, uint32_t miso_pin);

void CC1200_Init(void);

void CC1200_CommandStrobe(uint8_t strobe);

void CC1200_TransmitPacket(uint8_t *data, uint8_t length);

void CC1200_ReceiveHeader(uint8_t *buffer);
uint8_t CC1200_ReceivePayload(uint8_t *buffer, uint8_t length);

uint8_t CC1200_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* CC1200_H */