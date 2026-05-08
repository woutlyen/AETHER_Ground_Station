/****************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings
 *  Made by Wout Lyen
 *
 *  RF device: CC1200
 *
 ****************************************************************
 Settings for config:
 Address Config = No address check 
 Bit Rate = 1000 
 Carrier Frequency = 867.999878 
 Deviation = 399.169922 
 Device Address = 0 
 Manchester Enable = false 
 Modulation Format = 4-GFSK 
 Packet Bit Length = 0 
 Packet Length = 255 
 Packet Length Mode = Variable 
 RX Filter BW = 1666.666667 
 Symbol rate = 500 
 Whitening = false 
***************************************************************/

#include "CC1200_REG.h"

const CC1200_reg_t CC1200_cfg[] = {
{0x0000,0x07},    //IOCFG3            (GPIO3 IO Pin Configuration)
{0x0001,0x06},    //IOCFG2            (GPIO2 IO Pin Configuration)
{0x0003,0x00},    //IOCFG0            (GPIO0 IO Pin Configuration)
{0x0008,0xA8},    //SYNC_CFG1         (Sync Word Detection Configuration Reg. 1)
{0x000A,0x47},    //DEVIATION_M       (Frequency Deviation Configuration)
{0x000B,0x2F},    //MODCFG_DEV_E      (Modulation Format and Frequency Deviation Configur..)
{0x000C,0x1E},    //DCFILT_CFG        (Digital DC Removal Configuration)
{0x000E,0x8A},    //PREAMBLE_CFG0     (Preamble Detection Configuration Reg. 0)
{0x000F,0x00},    //IQIC              (Digital Image Channel Compensation Configuration)
{0x0010,0x01},    //CHAN_BW           (Channel Filter Configuration)
{0x0011,0x42},    //MDMCFG1           (General Modem Parameter Configuration Reg. 1)
{0x0012,0x05},    //MDMCFG0           (General Modem Parameter Configuration Reg. 0)
{0x0013,0xB9},    //SYMBOL_RATE2      (Symbol Rate Configuration Exponent and Mantissa [1..)
{0x0014,0x99},    //SYMBOL_RATE1      (Symbol Rate Configuration Mantissa [15:8])
{0x0015,0x9A},    //SYMBOL_RATE0      (Symbol Rate Configuration Mantissa [7:0])
{0x0016,0x2F},    //AGC_REF           (AGC Reference Level Configuration)
{0x0017,0xF8},    //AGC_CS_THR        (Carrier Sense Threshold Configuration)
{0x001A,0x60},    //AGC_CFG2          (Automatic Gain Control Configuration Reg. 2)
{0x001B,0x12},    //AGC_CFG1          (Automatic Gain Control Configuration Reg. 1)
{0x001C,0x84},    //AGC_CFG0          (Automatic Gain Control Configuration Reg. 0)
{0x001D,0x40},    //FIFO_CFG          (FIFO Configuration)
//{0x001F,0x1B},    //Frequency Synthesizer Calibration and Settling Con..
{0x0020,0x12},    //FS_CFG            (Frequency Synthesizer Configuration)
{0x0026,0x00},    //PKT_CFG2          (Packet Configuration Reg. 2)
{0x0027,0x03},    //PKT_CFG1          (Packet Configuration Reg. 1) //DISABLE CRC
{0x0028,0x20},    //PKT_CFG0          (Packet Configuration Reg. 0)
{0x002B,0x77},    //PA_CFG1           (Power Amplifier Configuration Reg. 1)
{0x002E,0xFF},    //PKT_LEN           (Packet Length Configuration)
{0x2F01,0x23},    //FREQOFF_CFG       (Frequency Offset Correction Configuration)
{0x2F05,0x00},    //MDMCFG2           (General Modem Parameter Configuration Reg. 2)
{0x2F0C,0x56},    //FREQ2             (Frequency Configuration [23:16])
{0x2F0D,0xCC},    //FREQ1             (Frequency Configuration [15:8])
{0x2F0E,0xCC},    //FREQ0             (Frequency Configuration [7:0])
{0x2F10,0xEE},    //IF_ADC1           (Analog to Digital Converter Configuration Reg. 1)
{0x2F11,0x10},    //IF_ADC0           (Analog to Digital Converter Configuration Reg. 0)
{0x2F12,0x04},    //FS_DIG1           (Frequency Synthesizer Digital Reg. 1)
{0x2F13,0xA3},    //FS_DIG0           (Frequency Synthesizer Digital Reg. 0)
{0x2F16,0x40},    //FS_CAL1           (Frequency Synthesizer Calibration Reg. 1)
{0x2F17,0x0E},    //FS_CAL0           (Frequency Synthesizer Calibration Reg. 0)
{0x2F19,0x03},    //FS_DIVTWO         (Frequency Synthesizer Divide by 2)
{0x2F1B,0x33},    //FS_DSM0           (FS Digital Synthesizer Module Configuration Reg. 0)
{0x2F1C,0xF7},    //FS_DVC1           (Frequency Synthesizer Divider Chain Configuration ..)
{0x2F1D,0x0F},    //FS_DVC0           (Frequency Synthesizer Divider Chain Configuration ..)
{0x2F1F,0x00},    //FS_PFD            (Frequency Synthesizer Phase Frequency Detector Con..)
{0x2F20,0x6E},    //FS_PRE            (Frequency Synthesizer Prescaler Configuration)
{0x2F21,0x1C},    //FS_REG_DIV_CML    (Frequency Synthesizer Divider Regulator Configurat..)
{0x2F22,0xAC},    //FS_SPARE          (Frequency Synthesizer Spare)
{0x2F27,0xB5},    //FS_VCO0           (FS Voltage Controlled Oscillator Configuration Reg..)
{0x2F2F,0x0D},    //IFAMP             (Intermediate Frequency Amplifier Configuration)
{0x2F32,0x0E},    //XOSC5             (Crystal Oscillator Configuration Reg. 5)
{0x2F36,0x03},    //XOSC1             (Crystal Oscillator Configuration Reg. 1)
};

const uint16_t CC1200_cfg_size = sizeof(CC1200_cfg) / sizeof(CC1200_reg_t);

