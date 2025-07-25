/**
 * @file disp.h
 * Nguyennhan
 */

#ifndef DISP_H
#define DISP_H

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include "lvgl/lvgl.h"
#include "bsp_lcd.h"

/*********************
 *      DEFINES
 *********************/
#define TFT_HOR_RES 240
#define TFT_VER_RES 320

#define TFT_EXT_FB		0		/*Frame buffer is located into an external SDRAM*/
#define TFT_USE_GPU		0		/*Enable hardware accelerator*/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void tft_init(void);

/**********************
 *      MACROS
 **********************/

#endif
