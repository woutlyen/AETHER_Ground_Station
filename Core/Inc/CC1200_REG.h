/****************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings
 *  Made by Wout Lyen
 *
 *  RF device: CC1200
 *
 ***************************************************************/

#ifndef CC1200_REG_H
#define CC1200_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint16_t addr;
    uint8_t value;
} CC1200_reg_t;

extern const CC1200_reg_t CC1200_cfg[];
extern const uint16_t CC1200_cfg_size;

#ifdef __cplusplus
}
#endif

#endif /* CC1200_REG */