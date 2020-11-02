/**
 * @file ili9488
 .c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9488.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9488_send_cmd(uint8_t cmd);
static void ili9488_send_data(void * data, uint16_t length);
static void ili9488_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9488_init(void)
{

#if defined (TFT_PARALLEL_8_BIT)
    #define INTERFACE_FORMAT 0x55 //16 bit colour for parallel    
    #error parallel
#else
    #define INTERFACE_FORMAT 0x66 //18 bit colour for SPI 
#endif
    
	lcd_init_cmd_t ili9488_init_cmds[]={
		{0xE0, {0x00,0x03,0x09,0x08,0x16,0x0A,0x3F,0x78,0x4C,0x09,0x0A,0x08,0x16,0x1A,0x0F}, 15},   // Positive Gamma Control
        {0XE1, {0x00,0x16,0x19,0x03,0x0F,0x05,0x32,0x45,0x46,0x04,0x0E,0x0D,0x35,0x37,0x0F}, 15},   // Negative Gamma Control
        {0XC0, {0x17,0x15}, 2},                                                                     // Power Control 1
        {0xC1, {0x41}, 1},                                                                          // Power Control 2
        {0xC5, {0x00,0x12,0x80}, 3},                                                                // VCOM Control
        {TFT_MADCTL, {0x48}, 1},                                                                    // Memory Access Control, MX, BGR
        {0x3A, {INTERFACE_FORMAT}, 1},                                                              // Pixel Interface Format
        {0xB0, {0x00}, 1},                                                                          // Interface Mode Control
        {0xB1, {0xA0}, 1},                                                                          // Frame Rate Control
        {0xB4, {0x02}, 1},                                                                          // Display Inversion Control
        {0xB6, {0x02,0x02,0x3B}, 3},                                                                // Display Function Control
        {0xB7, {0xC6}, 1},                                                                          // Entry Mode Set
        {0xF7, {0xA9,0x51,0x2C,0x82}, 4},                                                           // Adjust Control 3
		{TFT_SLPOUT, {0}, 0x80},
		{TFT_DISPON, {0}, 0x80},
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
	gpio_set_direction(ILI9488_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(ILI9488_RST, GPIO_MODE_OUTPUT);
	gpio_set_direction(ILI9488_BCKL, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(ILI9488_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(ILI9488_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

	printf("ILI9488 initialization.\n");

	//Send all the commands
	uint16_t cmd = 0;
	while (ili9488_init_cmds[cmd].databytes!=0xff) {
		ili9488_send_cmd(ili9488_init_cmds[cmd].cmd);
		ili9488_send_data(ili9488_init_cmds[cmd].data, ili9488_init_cmds[cmd].databytes&0x1F);
		if (ili9488_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	///Enable backlight
	printf("Enable backlight.\n");
	gpio_set_level(ILI9488_BCKL, CONFIG_DISP_BCKL_ACTIVE_LVL);
    
    printf("dc: %d, rst: %d, bckl: %d, clk: %d, mosi: %d, miso: %d\n", ILI9488_DC, ILI9488_RST, ILI9488_BCKL, CONFIG_DISP_SPI_CLK, CONFIG_DISP_SPI_MOSI, CONFIG_DISP_SPI_MISO);
}


static void *convert16to18bit(uint32_t idx, void *data, uint16_t length) {
   
    static lv_color_t buf1[DISP_BUF_SIZE/2*3];
    static lv_color_t buf2[DISP_BUF_SIZE/2*3]; 
    
    uint8_t *buf = idx ? (uint8_t *)buf1 : (uint8_t *)buf2;
    
    uint16_t i = 0;
    uint16_t *pData = (uint16_t*)data;
    
#ifdef LV_COLOR_16_SWAP    
    for(uint16_t j = 0; i < length; i += 2) {
        buf[j++] = ((*pData) & 0xF800)>>8;
        buf[j++] = ((*pData) & 0x07E0)>>3;
        buf[j++] = ((*pData) & 0x001F)<<3;
        ++pData;
    }
#else
    for(uint16_t j = 0; i < length; i += 2) {
        buf[j++] = (*pData) & 0xF8;
        buf[j++] = ((*pData) & 0xE000)>>11 | ((*pData) & 0x07)<<5;
        buf[j++] = ((*pData) & 0x1F00)>>5;
        ++pData;
    }
#endif

   return buf;
}


void ili9488_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
	uint8_t data[4];

	/*Column addresses*/
	ili9488_send_cmd(0x2A);
	data[0] = (area->x1 >> 8) & 0xFF;
	data[1] = area->x1 & 0xFF;
	data[2] = (area->x2 >> 8) & 0xFF;
	data[3] = area->x2 & 0xFF;
	ili9488_send_data(data, 4);

	/*Page addresses*/
	ili9488_send_cmd(0x2B);
	data[0] = (area->y1 >> 8) & 0xFF;
	data[1] = area->y1 & 0xFF;
	data[2] = (area->y2 >> 8) & 0xFF;
	data[3] = area->y2 & 0xFF;
	ili9488_send_data(data, 4);

	/*Memory write*/
	ili9488_send_cmd(0x2C);


	uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
    
    assert(16 == LV_COLOR_DEPTH && "Currently only 16bit mode is suppoted");
    void *pBuff = convert16to18bit(drv->buffer->buf1 == color_map, (void*)color_map, size * 2);
	ili9488_send_color(pBuff, size * 3);
}







///////////////////////// todo.. move it to the seperate file ////////////////////////////////

static uint16_t getTouchRawZ() {

  // Z sample request
  int16_t tz = 0xFFF;

  tz += disp_spi_send_16(0xB0B0) >> 3;  // Read Z1 
  tz -= disp_spi_send_16(0xC0C0) >> 3;  // Read Z2

  //printf("tz: %u\n", (uint16_t)tz);

  return (uint16_t)tz;
}

static void getTouchRaw(uint16_t *x, uint16_t *y) {
    
    *x = disp_spi_send_16(0xD0D0) >> 3;
    *y = disp_spi_send_16(0x9090) >> 3;
}

static void convertRawXY(uint16_t *x, uint16_t *y)
{
    *x = *x*CONFIG_DISP_HOR_RES/4096;
    *y = CONFIG_DISP_VER_RES - *y*CONFIG_DISP_VER_RES/4096;
}

bool xpt2046_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
    assert(0 != data);
    
    static uint16_t x = 0;
    static uint16_t y = 0;
  
    uint16_t z = getTouchRawZ();
    if(300 < z) {
        getTouchRaw(&x, &y);
        convertRawXY(&x, &y);
        data->state = LV_INDEV_STATE_PR;
    }
    else {
        data->state = LV_INDEV_STATE_REL;
    }
        
    data->point.x = x;
    data->point.y = y;    
    
    //printf("x: %d, y: %d\n", x, y);
    return 0;
}
///////////////////////// todo.. move it to the seperate file ////////////////////////////////









/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ili9488_send_cmd(uint8_t cmd)
{
	gpio_set_level(ILI9488_DC, 0);	 /*Command mode*/
	disp_spi_send_data(&cmd, 1);
}

static void ili9488_send_data(void * data, uint16_t length)
{
	gpio_set_level(ILI9488_DC, 1);	 /*Data mode*/
	disp_spi_send_data(data, length);
}

static void ili9488_send_color(void * data, uint16_t length)
{
    gpio_set_level(ILI9488_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}
