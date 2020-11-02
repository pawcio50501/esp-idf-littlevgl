/**
 * @file disp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_spi.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "freertos/task.h"
#include "lvgl/lvgl.h"
#include "st7789.h"
#include "sdkconfig.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR spi_ready (spi_transaction_t *trans);

/**********************
 *  STATIC VARIABLES
 **********************/
static spi_device_handle_t spi;
static spi_device_handle_t spiTouch;
static volatile bool spi_trans_in_progress;
static volatile bool spi_color_sent;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void disp_spi_init(void)
{

    esp_err_t ret;

    spi_bus_config_t buscfg={
            .miso_io_num=CONFIG_DISP_SPI_MISO,
            .mosi_io_num=DISP_SPI_MOSI,
            .sclk_io_num=DISP_SPI_CLK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz = DISP_BUF_SIZE * 3, // defined currently in ili9488.h - which is silly!
    };

    spi_device_interface_config_t devcfg={
            .clock_speed_hz=CONFIG_DISP_SPI_SPEED*1000*1000,           //Clock out at 40 MHz
            .mode=0,                                //SPI mode 0
            .spics_io_num=DISP_SPI_CS,              //CS pin
            .queue_size=1,
            .pre_cb=NULL,
            .post_cb=spi_ready,
            .flags = SPI_DEVICE_HALFDUPLEX
    };

    spi_device_interface_config_t devcfgTouch={
            .clock_speed_hz=CONFIG_DISP_SPI_TOUCH_SPEED*1000,           //Clock out at 2.5 MHz
            .mode=0,                           //SPI mode 0
            .spics_io_num=CONFIG_DISP_SPI_TOUCH_CS,                  //CS touch pin 
            .queue_size=1,
            .pre_cb=NULL,
            .post_cb=NULL,
            .flags = 0
    };
    
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    
    //Attach the Touch to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfgTouch, &spiTouch);
    assert(ret==ESP_OK);    
}

void disp_spi_send_data(uint8_t * data, uint16_t length)
{
    if (length == 0) return;           //no need to send anything

    while(spi_trans_in_progress);

    spi_transaction_t t = {
        .length = length * 8, // transaction length is in bits
        .tx_buffer = data
    };

    spi_trans_in_progress = true;
    spi_color_sent = false;             //Mark the "lv_flush_ready" NOT needs to be called in "spi_ready"
    spi_device_queue_trans(spi, &t, portMAX_DELAY);

}

void disp_spi_send_colors(uint8_t * data, uint16_t length)
{
    if (length == 0) return;           //no need to send anything

    while(spi_trans_in_progress);

    spi_transaction_t t = {
        .length = length * 8, // transaction length is in bits
        .tx_buffer = data
    };
    
    spi_trans_in_progress = true;
    spi_color_sent = true;              //Mark the "lv_flush_ready" needs to be called in "spi_ready"
    spi_device_queue_trans(spi, &t, portMAX_DELAY);
}

static uint8_t disp_spi_send_impl(uint8_t data)
{
    spi_transaction_t r = {
        .length = 8, 
        .tx_data = {data, 0, 0, 0},
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };
    
    /*esp_err_t err =*/ spi_device_transmit(spiTouch, &r);
    //printf("rx err: %#x, rxlen: %d, ret: %d\n", err, r.rxlength, r.rx_data[0]);
    
    return r.rx_data[0];
}

uint8_t disp_spi_send(uint8_t data)
{
    while(spi_trans_in_progress);
    spi_trans_in_progress = true;
    uint8_t r = disp_spi_send_impl(data);
    spi_trans_in_progress = false;
    return r;
}

uint16_t disp_spi_send_16(uint16_t data)
{
    while(spi_trans_in_progress);

    spi_trans_in_progress = true;
    uint8_t h = disp_spi_send_impl((uint8_t)(data >> 8));
    uint8_t l = disp_spi_send_impl((uint8_t)(data & 0xff));
   
    //printf("h: %d, l: %d\n", h, l);
    
    spi_trans_in_progress = false;
    
    return (((uint16_t)h) << 8) | l;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void IRAM_ATTR spi_ready (spi_transaction_t *trans)
{
    spi_trans_in_progress = false;

    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    if(spi_color_sent) lv_disp_flush_ready(&disp->driver);
}
