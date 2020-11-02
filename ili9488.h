/**
 * @file ili9488.h
 *
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl/lvgl.h"
#include "sdkconfig.h"

/*********************
 *      DEFINES
 *********************/
#define DISP_BUF_SIZE (LV_HOR_RES_MAX * 16)
#define ILI9488_DC CONFIG_DISP_DC
#define ILI9488_RST CONFIG_DISP_RST
#define ILI9488_BCKL CONFIG_DISP_BCKL

// Change the width and height if required (defined in portrait mode)
// or use the constructor to over-ride defaults
#define TFT_WIDTH  CONFIG_DISP_HOR_RES
#define TFT_HEIGHT CONFIG_DISP_VER_RES

// Delay between some initialisation commands
#define TFT_INIT_DELAY 0x80 // Not used unless commandlist invoked


// Generic commands 
#define TFT_NOP     0x00
#define TFT_SWRST   0x01

#define TFT_SLPIN   0x10
#define TFT_SLPOUT  0x11

#define TFT_INVOFF  0x20
#define TFT_INVON   0x21

#define TFT_DISPOFF 0x28
#define TFT_DISPON  0x29

#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C

#define TFT_RAMRD   0x2E

#define TFT_MADCTL  0x36

#define TFT_MAD_MY  0x80
#define TFT_MAD_MX  0x40
#define TFT_MAD_MV  0x20
#define TFT_MAD_ML  0x10
#define TFT_MAD_RGB 0x00
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH  0x04
#define TFT_MAD_SS  0x02
#define TFT_MAD_GS  0x01

#define TFT_IDXRD   0x00 // ILI9341 only, indexed control register read

 /**********************
 *      TYPEDEFS
 **********************/

 /**********************
 * GLOBAL PROTOTYPES
 **********************/

    void ili9488_init(void);
    void ili9488_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
    bool xpt2046_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data);


 /**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif


