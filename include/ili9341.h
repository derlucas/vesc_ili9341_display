#ifndef ILI9341_H_
#define ILI9341_H_

/**
ILI9341      STM32F10x    DESCRIPTION
		
SDO (MISO)   PA6          Output from LCD for SPI.	Not used, can be left
LED          3.3V         Backlight
SCK          PA5          SPI clock
SDI (MOSI)   PA7          SPI master output
WRX or D/C   PA2          Data/Command register
RESET        PA3          Reset LCD
CS           PA4          Chip select for SPI
GND          GND          Ground
VCC          3.3V         Positive power supply
*/

#include "stm32f10x_conf.h"
#include "ili9341_fonts.h"
#include "display_spi.h"


#define ILI9341_SPI             SPI1
#define ILI9341_CS_PIN          GPIO_Pin_4
#define ILI9341_WRX_PIN         GPIO_Pin_2
#define ILI9341_RST_PIN         GPIO_Pin_3
#define ILI9341_LED_PIN         GPIO_Pin_0

/* LCD settings */
#define ILI9341_WIDTH           240
#define ILI9341_HEIGHT          320
#define ILI9341_PIXEL           76800

/* Colors */
#define ILI9341_COLOR_WHITE			0xFFFF
#define ILI9341_COLOR_BLACK			0x0000
#define ILI9341_COLOR_RED           0xF800
#define ILI9341_COLOR_GREEN			0x07E0
#define ILI9341_COLOR_GREEN2		0xB723
#define ILI9341_COLOR_BLUE			0x001F
#define ILI9341_COLOR_BLUE2			0x051D
#define ILI9341_COLOR_YELLOW		0xFFE0
#define ILI9341_COLOR_ORANGE		0xFBE4
#define ILI9341_COLOR_CYAN			0x07FF
#define ILI9341_COLOR_MAGENTA		0xA254
#define ILI9341_COLOR_GRAY			0x7BEF
#define ILI9341_COLOR_BROWN			0xBBCA

/* Transparent background, only for strings and chars */
#define ILI9341_TRANSPARENT			0x80000000


typedef enum {
	ILI9341_Orientation_Portrait_1,
	ILI9341_Orientation_Portrait_2,
	ILI9341_Orientation_Landscape_1,
	ILI9341_Orientation_Landscape_2
} ILI9341_Orientation_t;



void ILI9341_Init(void);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint32_t color);
void ILI9341_Fill(uint32_t color);
void ILI9341_Rotate(ILI9341_Orientation_t orientation);

void ILI9341_Putc(uint16_t x, uint16_t y, char c, FontDef_t* font, uint32_t foreground, uint32_t background);
void ILI9341_Puts(uint16_t x, uint16_t y, char* str, FontDef_t *font, uint32_t foreground, uint32_t background);
void ILI9341_GetStringSize(char* str, FontDef_t* font, uint16_t* width, uint16_t* height);

void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);
void ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);
void ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);
void ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);
void ILI9341_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);

void ILI9341_DisplayOn(void);
void ILI9341_DisplayOff(void);

void ILI9341_LEDOn(void);
void ILI9341_LEDOff(void);


#endif

