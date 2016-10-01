/**
 * based on code from Tilen Majerle, 2014 / GPL
 * TM_ prefixes removed, some changes: Lucas Pleß, 2016
 */
#include "stm32f10x_conf.h"
#include "ili9341.h"

extern uint8_t Font7seg[];

typedef enum {
	ILI9341_Landscape,
	ILI9341_Portrait
} ILI9341_Orientation;

typedef struct {
	uint16_t width;
	uint16_t height;
	ILI9341_Orientation orientation; // 1 = portrait; 0 = landscape
} ILI931_Options_t;

/* Pin definitions */
#define ILI9341_RST_SET				GPIO_SetBits(GPIOA, ILI9341_RST_PIN)
#define ILI9341_RST_RESET			GPIO_ResetBits(GPIOA, ILI9341_RST_PIN)
#define ILI9341_CS_SET				GPIO_SetBits(GPIOA, ILI9341_CS_PIN)
#define ILI9341_CS_RESET			GPIO_ResetBits(GPIOA, ILI9341_CS_PIN)
#define ILI9341_WRX_SET				GPIO_SetBits(GPIOA, ILI9341_WRX_PIN)
#define ILI9341_WRX_RESET			GPIO_ResetBits(GPIOA, ILI9341_WRX_PIN)
#define ILI9341_LED_SET             GPIO_SetBits(GPIOB, ILI9341_LED_PIN)
#define ILI9341_LED_RESET           GPIO_ResetBits(GPIOB, ILI9341_LED_PIN)

/* Private defines */
#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MAC					0x36
#define ILI9341_PIXEL_FORMAT		0x3A
#define ILI9341_WDB					0x51
#define ILI9341_WCD					0x53
#define ILI9341_RGB_INTERFACE		0xB0
#define ILI9341_FRC					0xB1
#define ILI9341_BPC					0xB5
#define ILI9341_DFC					0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC					0xF7

/* Pin functions */
uint16_t ILI9341_x;
uint16_t ILI9341_y;
ILI931_Options_t ILI9341_Opts;
uint8_t ILI9341_INT_CalledFromPuts = 0;


/* Private functions */
void ILI9341_InitLCD(void);
void ILI9341_SendData(uint8_t data);
void ILI9341_SendCommand(uint8_t data);
void ILI9341_Delay(volatile unsigned int delay);
void ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9341_INT_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);


void ILI9341_Init() {
    // enable clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
            RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // DC/RS, CS
	GPIO_InitStructure.GPIO_Pin = ILI9341_WRX_PIN | ILI9341_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // LED backlight
    GPIO_InitStructure.GPIO_Pin = ILI9341_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	

    // MOSI & SCK
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // SPI will run at 36MHz


	ILI9341_CS_SET;
	
	display_spi_init(SPI1);

	
	ILI9341_LED_RESET;

	/* first init LCD */
	ILI9341_InitLCD();
	
	/* Set default settings */
	ILI9341_x = ILI9341_y = 0;
	ILI9341_Opts.width = ILI9341_WIDTH;
	ILI9341_Opts.height = ILI9341_HEIGHT;
	ILI9341_Opts.orientation = ILI9341_Portrait;
	
	/* Fill with white color */
	ILI9341_Fill(ILI9341_COLOR_BLACK);

	ILI9341_LED_SET;
}

void ILI9341_LEDOn(void) {
    ILI9341_LED_SET;
}
void ILI9341_LEDOff(void) {
    ILI9341_LED_RESET;
}

void TM_ILI9341_Delay(volatile unsigned int delay) {
	for (; delay != 0; delay--);
}

void ILI9341_InitLCD(void) {
	/* Force reset */
	ILI9341_RST_RESET;
	TM_ILI9341_Delay(20000);
	ILI9341_RST_SET;
	
	/* Delay for RST response */
	TM_ILI9341_Delay(20000);
	
	/* Software reset */
	ILI9341_SendCommand(ILI9341_RESET);
	ILI9341_Delay(50000);
	
	ILI9341_SendCommand(ILI9341_POWERA);
	ILI9341_SendData(0x39);
	ILI9341_SendData(0x2C);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x34);
	ILI9341_SendData(0x02);
	ILI9341_SendCommand(ILI9341_POWERB);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0xC1);
	ILI9341_SendData(0x30);
	ILI9341_SendCommand(ILI9341_DTCA);
	ILI9341_SendData(0x85);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x78);
	ILI9341_SendCommand(ILI9341_DTCB);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendCommand(ILI9341_POWER_SEQ);
	ILI9341_SendData(0x64);
	ILI9341_SendData(0x03);
	ILI9341_SendData(0x12);
	ILI9341_SendData(0x81);
	ILI9341_SendCommand(ILI9341_PRC);
	ILI9341_SendData(0x20);
	ILI9341_SendCommand(ILI9341_POWER1);
	ILI9341_SendData(0x23);
	ILI9341_SendCommand(ILI9341_POWER2);
	ILI9341_SendData(0x10);
	ILI9341_SendCommand(ILI9341_VCOM1);
	ILI9341_SendData(0x3E);
	ILI9341_SendData(0x28);
	ILI9341_SendCommand(ILI9341_VCOM2);
	ILI9341_SendData(0x86);
	ILI9341_SendCommand(ILI9341_MAC);
	ILI9341_SendData(0x48);
	ILI9341_SendCommand(ILI9341_PIXEL_FORMAT);
	ILI9341_SendData(0x55);
	ILI9341_SendCommand(ILI9341_FRC);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x18);
	ILI9341_SendCommand(ILI9341_DFC);
	ILI9341_SendData(0x08);
	ILI9341_SendData(0x82);
	ILI9341_SendData(0x27);
	ILI9341_SendCommand(ILI9341_3GAMMA_EN);
	ILI9341_SendData(0x00);
	ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0xEF);
	ILI9341_SendCommand(ILI9341_PAGE_ADDR);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x01);
	ILI9341_SendData(0x3F);
	ILI9341_SendCommand(ILI9341_GAMMA);
	ILI9341_SendData(0x01);
	ILI9341_SendCommand(ILI9341_PGAMMA);
	ILI9341_SendData(0x0F);
	ILI9341_SendData(0x31);
	ILI9341_SendData(0x2B);
	ILI9341_SendData(0x0C);
	ILI9341_SendData(0x0E);
	ILI9341_SendData(0x08);
	ILI9341_SendData(0x4E);
	ILI9341_SendData(0xF1);
	ILI9341_SendData(0x37);
	ILI9341_SendData(0x07);
	ILI9341_SendData(0x10);
	ILI9341_SendData(0x03);
	ILI9341_SendData(0x0E);
	ILI9341_SendData(0x09);
	ILI9341_SendData(0x00);
	ILI9341_SendCommand(ILI9341_NGAMMA);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x0E);
	ILI9341_SendData(0x14);
	ILI9341_SendData(0x03);
	ILI9341_SendData(0x11);
	ILI9341_SendData(0x07);
	ILI9341_SendData(0x31);
	ILI9341_SendData(0xC1);
	ILI9341_SendData(0x48);
	ILI9341_SendData(0x08);
	ILI9341_SendData(0x0F);
	ILI9341_SendData(0x0C);
	ILI9341_SendData(0x31);
	ILI9341_SendData(0x36);
	ILI9341_SendData(0x0F);
	ILI9341_SendCommand(ILI9341_SLEEP_OUT);

	ILI9341_Delay(1000000);

	ILI9341_SendCommand(ILI9341_DISPLAY_ON);
	ILI9341_SendCommand(ILI9341_GRAM);
}

void ILI9341_DisplayOn(void) {
	ILI9341_SendCommand(ILI9341_DISPLAY_ON);
}

void ILI9341_DisplayOff(void) {
	ILI9341_SendCommand(ILI9341_DISPLAY_OFF);
}

void ILI9341_SendCommand(uint8_t data) {
	ILI9341_WRX_RESET;
	ILI9341_CS_RESET;
	display_spi_send(ILI9341_SPI, data);
	ILI9341_CS_SET;
}

void ILI9341_SendData(uint8_t data) {
	ILI9341_WRX_SET;
	ILI9341_CS_RESET;
	display_spi_send(ILI9341_SPI, data);
	ILI9341_CS_SET;
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint32_t color) {
	ILI9341_SetCursorPosition(x, y, x, y);

	ILI9341_SendCommand(ILI9341_GRAM);
	ILI9341_SendData(color >> 8);
	ILI9341_SendData(color & 0xFF);
}


void ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
	ILI9341_SendData(x1 >> 8);
	ILI9341_SendData(x1 & 0xFF);
	ILI9341_SendData(x2 >> 8);
	ILI9341_SendData(x2 & 0xFF);

	ILI9341_SendCommand(ILI9341_PAGE_ADDR);
	ILI9341_SendData(y1 >> 8);
	ILI9341_SendData(y1 & 0xFF);
	ILI9341_SendData(y2 >> 8);
	ILI9341_SendData(y2 & 0xFF);
}

void ILI9341_Fill(uint32_t color) {
	ILI9341_INT_Fill(0, 0, ILI9341_Opts.width - 1, ILI9341_Opts.height, color);
}

void ILI9341_INT_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
	uint32_t pixels_count;
	
	/* Set cursor position */
	ILI9341_SetCursorPosition(x0, y0, x1, y1);

	/* Set command for GRAM data */
	ILI9341_SendCommand(ILI9341_GRAM);
	
	/* Calculate pixels count */
	pixels_count = (x1 - x0 + 1) * (y1 - y0 + 1);

	/* Send everything */
	ILI9341_CS_RESET;
	ILI9341_WRX_SET;
	
	/* Go to 16-bit SPI mode */
	display_spi_setdatasize(ILI9341_SPI, SPI_DataSize_16b);
	
	/* Send first 65535 bytes, SPI MUST BE IN 16-bit MODE */
	display_spi_dma_sendhalfword(color, (pixels_count > 0xFFFF) ? 0xFFFF : pixels_count);
	/* Wait till done */
	while (display_spi_dma_working(ILI9341_SPI));
	
	/* Check again */
	if (pixels_count > 0xFFFF) {
		/* Send remaining data */
	    display_spi_dma_sendhalfword(color, pixels_count - 0xFFFF);
		/* Wait till done */
		while (display_spi_dma_working(ILI9341_SPI));
	}
	
	ILI9341_CS_SET;

	/* Go back to 8-bit SPI mode */
	display_spi_setdatasize(ILI9341_SPI, SPI_DataSize_8b);
}

void ILI9341_Delay(volatile unsigned int delay) {
	for (; delay != 0; delay--); 
}

void ILI9341_Rotate(ILI9341_Orientation_t orientation) {
    ILI9341_SendCommand(ILI9341_MAC);
	if (orientation == ILI9341_Orientation_Portrait_1) {
		ILI9341_SendData(0x58);
	} else if (orientation == ILI9341_Orientation_Portrait_2) {
		ILI9341_SendData(0x88);
	} else if (orientation == ILI9341_Orientation_Landscape_1) {
		ILI9341_SendData(0x28);
	} else if (orientation == ILI9341_Orientation_Landscape_2) {
		ILI9341_SendData(0xE8);
	}
	
	if (orientation == ILI9341_Orientation_Portrait_1 ||
	    orientation == ILI9341_Orientation_Portrait_2) {
		ILI9341_Opts.width = ILI9341_WIDTH;
		ILI9341_Opts.height = ILI9341_HEIGHT;
		ILI9341_Opts.orientation = ILI9341_Portrait;
	} else {
		ILI9341_Opts.width = ILI9341_HEIGHT;
		ILI9341_Opts.height = ILI9341_WIDTH;
		ILI9341_Opts.orientation = ILI9341_Landscape;
	}
}

void ILI9341_Puts(uint16_t x, uint16_t y, char *str, FontDef_t *font, uint32_t foreground, uint32_t background) {
	uint16_t startX = x;
	
	/* Set X and Y coordinates */
	ILI9341_x = x;
	ILI9341_y = y;
	
	while (*str) {
		/* New line */
		if (*str == '\n') {
			ILI9341_y += font->FontHeight + 1;
			/* if after \n is also \r, than go to the left of the screen */
			if (*(str + 1) == '\r') {
				ILI9341_x = 0;
				str++;
			} else {
				ILI9341_x = startX;
			}
			str++;
			continue;
		} else if (*str == '\r') {
			str++;
			continue;
		}
		
		/* Put character to LCD */
		ILI9341_Putc(ILI9341_x, ILI9341_y, *str++, font, foreground, background);
	}
}

void ILI9341_GetStringSize(char *str, FontDef_t *font, uint16_t *width, uint16_t *height) {
	uint16_t w = 0;
	*height = font->FontHeight;
	while (*str++) {
		w += font->FontWidth;
	}
	*width = w;
}

void ILI9341_Putc(uint16_t x, uint16_t y, char c, FontDef_t *font, uint32_t foreground, uint32_t background) {
	uint32_t i, b, j, z;
	uint8_t ch;
	/* Set coordinates */
	ILI9341_x = x;
	ILI9341_y = y;
	
	if ((ILI9341_x + font->FontWidth) > ILI9341_Opts.width) {
		/* If at the end of a line of display, go to new line and set x to 0 position */
		ILI9341_y += font->FontHeight;
		ILI9341_x = 0;
	}
	
	/* Draw rectangle for background */
	ILI9341_INT_Fill(ILI9341_x, ILI9341_y, ILI9341_x + font->FontWidth, ILI9341_y + font->FontHeight, background);
	
	/* Draw font data TM fonts*/
	if(font != &Font_7seg) {
		for (i = 0; i < font->FontHeight; i++) {
			b = font->data[(c - 32) * font->FontHeight + i];
			for (j = 0; j < font->FontWidth; j++) {
				if ((b << j) & 0x8000) {
					ILI9341_DrawPixel(ILI9341_x + j, (ILI9341_y + i), foreground);
				}
			}
		}
	} else {
		// 7-segment font from UTFT
		if(c >= 48 && c <=57) {
			b = (c-48)* font->FontWidth/8 * font->FontHeight;

			for (j = 0; j < font->FontHeight; j++) {
				// j is done 50 times (for every line in 32x50 font)

				for(z = 0; z < 4; z++) {	// the font has 4 bytes per line
					ch = Font7seg[b];		// using the font directly (hack) because addressing is 16bit but font comes in 8bit

					for (i = 0; i < 8; i++) {

						if((ch & (1 << (7-i))) != 0) {

							ILI9341_DrawPixel(ILI9341_x + z*8 + i, ILI9341_y + j, foreground);

						}
					}

					b++;
				}
			}
		}
	}
	
	/* Set new pointer */
	ILI9341_x += font->FontWidth;
}

void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) {
	int16_t dx, dy, sx, sy, err, e2; 	
	uint16_t tmp;
	
	/* Check for overflow */
	if (x0 >= ILI9341_Opts.width) {
		x0 = ILI9341_Opts.width - 1;
	}
	if (x1 >= ILI9341_Opts.width) {
		x1 = ILI9341_Opts.width - 1;
	}
	if (y0 >= ILI9341_Opts.height) {
		y0 = ILI9341_Opts.height - 1;
	}
	if (y1 >= ILI9341_Opts.height) {
		y1 = ILI9341_Opts.height - 1;
	}
	
	/* Check correction */
	if (x0 > x1) {
		tmp = x0;
		x0 = x1;
		x1 = tmp;
	}
	if (y0 > y1) {
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}
	
	dx = x1 - x0;
	dy = y1 - y0;
	
	/* Vertical or horizontal line */
	if (dx == 0 || dy == 0) {
		ILI9341_INT_Fill(x0, y0, x1, y1, color);
		return;
	}
	
	sx = (x0 < x1) ? 1 : -1; 
	sy = (y0 < y1) ? 1 : -1; 
	err = ((dx > dy) ? dx : -dy) / 2; 

	while (1) {
		ILI9341_DrawPixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err; 
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		} 
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		} 
	}
}

void ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) {
	ILI9341_DrawLine(x0, y0, x1, y0, color); //Top
	ILI9341_DrawLine(x0, y0, x0, y1, color);	//Left
	ILI9341_DrawLine(x1, y0, x1, y1, color);	//Right
	ILI9341_DrawLine(x0, y1, x1, y1, color);	//Bottom
}

void ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) {
	uint16_t tmp;
	
	/* Check correction */
	if (x0 > x1) {
		tmp = x0;
		x0 = x1;
		x1 = tmp;
	}
	if (y0 > y1) {
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}
	
	/* Fill rectangle */
	ILI9341_INT_Fill(x0, y0, x1, y1, color);
	
	/* CS HIGH back */
	ILI9341_CS_SET;
}

void ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ILI9341_DrawPixel(x0, y0 + r, color);
    ILI9341_DrawPixel(x0, y0 - r, color);
    ILI9341_DrawPixel(x0 + r, y0, color);
    ILI9341_DrawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ILI9341_DrawPixel(x0 + x, y0 + y, color);
        ILI9341_DrawPixel(x0 - x, y0 + y, color);
        ILI9341_DrawPixel(x0 + x, y0 - y, color);
        ILI9341_DrawPixel(x0 - x, y0 - y, color);

        ILI9341_DrawPixel(x0 + y, y0 + x, color);
        ILI9341_DrawPixel(x0 - y, y0 + x, color);
        ILI9341_DrawPixel(x0 + y, y0 - x, color);
        ILI9341_DrawPixel(x0 - y, y0 - x, color);
    }
}

void ILI9341_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ILI9341_DrawPixel(x0, y0 + r, color);
    ILI9341_DrawPixel(x0, y0 - r, color);
    ILI9341_DrawPixel(x0 + r, y0, color);
    ILI9341_DrawPixel(x0 - r, y0, color);
    ILI9341_DrawLine(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ILI9341_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        ILI9341_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

        ILI9341_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        ILI9341_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
    }
}


