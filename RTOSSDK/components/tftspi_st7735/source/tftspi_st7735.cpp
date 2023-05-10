/*
 * tftspi_st7735.cpp
 *
 *  Created on: 3 thg 6, 2022
 *      Author: A315-56
 */


#include "tftspi_st7735/tftspi_st7735.h"

#include "malloc.h"
#include "string.h"

#include "periph/dma.h"
#include "periph/gpio.h"
#include "periph/systick.h"


volatile uint8_t dma_tft_flag = 0;
volatile int8_t dma_tft_cnt = 0;

static uint16_t width, height;

static const uint8_t init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
	15,                       // 15 commands in list:
	ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
	  150,                    //     150 ms delay
	ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
	  255,                    //     500 ms delay
	ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
	  0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
	  0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
	  0x01, 0x2C, 0x2D,       //     Dot inversion mode
	  0x01, 0x2C, 0x2D,       //     Line inversion mode
	ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
	  0x07,                   //     No inversion
	ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
	  0xA2,
	  0x02,                   //     -4.6V
	  0x84,                   //     AUTO mode
	ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
	  0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
	ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
	  0x0A,                   //     Opamp current small
	  0x00,                   //     Boost frequency
	ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
	  0x8A,                   //     BCLK/2, Opamp current small & Medium low
	  0x2A,
	ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
	  0x8A, 0xEE,
	ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
	  0x0E,
	ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
	ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
	  ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
	ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
	  0x05 },                 //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
	2,                        //  2 commands in list:
	ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
	  0x00, 0x00,             //     XSTART = 0
	  0x00, 0x7F,             //     XEND = 127
	ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
	  0x00, 0x00,             //     XSTART = 0
	  0x00, 0x7F },           //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
	3,                        //  3 commands in list:
	ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
	  0x00, 0x00,             //     XSTART = 0
	  0x00, 0x4F,             //     XEND = 79
	ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
	  0x00, 0x00,             //     XSTART = 0
	  0x00, 0x9F ,            //     XEND = 159
	ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
	4,                        //  4 commands in list:
	ST7735_GMCTRP1, 16      , //  1: Gamma Adjustments (pos. polarity), 16 args, no delay:
	  0x02, 0x1c, 0x07, 0x12,
	  0x37, 0x32, 0x29, 0x2d,
	  0x29, 0x25, 0x2B, 0x39,
	  0x00, 0x01, 0x03, 0x10,
	ST7735_GMCTRN1, 16      , //  2: Gamma Adjustments (neg. polarity), 16 args, no delay:
	  0x03, 0x1d, 0x07, 0x06,
	  0x2E, 0x2C, 0x29, 0x2D,
	  0x2E, 0x2E, 0x37, 0x3F,
	  0x00, 0x00, 0x02, 0x10,
	ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
	  10,                     //     10 ms delay
	ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
	  100  					  //     100 ms delay
};
#ifdef TFT_CSPIN_EN
TFTSPI_ST7735::TFTSPI_ST7735(GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *rst_port, uint16_t rst_pin, GPIO_TypeDef *dc_port, uint16_t dc_pin){
	this->cs_port  = cs_port;
	this->rst_port = rst_port;
	this->dc_port  = dc_port;

	this->cs_pin  = cs_pin;
	this->rst_pin = rst_pin;
	this->dc_pin  = dc_pin;
	X_Start = 0, Y_Start = 0;
}
#endif
TFTSPI_ST7735::TFTSPI_ST7735(GPIO_TypeDef *rst_port, uint16_t rst_pin, GPIO_TypeDef *dc_port, uint16_t dc_pin){
	this->rst_port = rst_port;
	this->dc_port  = dc_port;

	this->rst_pin = rst_pin;
	this->dc_pin  = dc_pin;
	X_Start = 0, Y_Start = 0;
}
#ifdef TFT_CSPIN_EN
void TFTSPI_ST7735::Active(void) {
	gpio_reset(cs_port, cs_pin);
}

void TFTSPI_ST7735::Idle(void) {
	gpio_set(cs_port, cs_pin);
}
#endif

void TFTSPI_ST7735::EnableWrite(void){
	gpio_set(dc_port, dc_pin);
}

void TFTSPI_ST7735::Reset(void) {
	gpio_reset(rst_port, rst_pin);
	delay_ms(5);
	gpio_set(rst_port, rst_pin);
}

void TFTSPI_ST7735::WriteCommand(uint8_t cmd) {
	uint8_t tmp = cmd;
	gpio_reset(dc_port, dc_pin);
	_spi -> transmit((uint32_t)(&tmp), 1);
	(void)tmp;
}

void TFTSPI_ST7735::WriteData(uint8_t* buff, size_t buff_size) {
	EnableWrite();
	_spi -> transmit((uint32_t)buff, (int)buff_size);
}

void TFTSPI_ST7735::InitCommandList(const uint8_t *addr) {
	uint8_t numCommands, numArgs;
	uint16_t ms;

	numCommands = *addr++;
	while(numCommands--) {
		uint8_t cmd = *addr++;
		WriteCommand(cmd);
		numArgs = *addr++;
		ms = numArgs & DELAY;
		numArgs &= ~DELAY;
		if(numArgs) {
			WriteData((uint8_t*)addr, numArgs);
			addr += numArgs;
		}
		if(ms) {
			ms = *addr++;
			if(ms == 255) ms = 500;
			delay_ms(ms);
		}
	}
}

void TFTSPI_DMA_Handler(dma_event_t event, void *);

void TFTSPI_ST7735::init(spi_t spi, tftspi_colorformat_t format) {
	_format = format;
	_spi = spi;
	_spi->_txdma->register_event_handler(TFTSPI_DMA_Handler, NULL);
	/* GPIO FOR TFT LCD */
#ifdef TFT_CSPIN_EN
	gpio_set_mode(cs_port, cs_pin, GPIO_OUTPUT_PUSHPULL);
#endif
	gpio_set_mode(rst_port, rst_pin, GPIO_OUTPUT_PUSHPULL);
	gpio_set_mode(dc_port, dc_pin, GPIO_OUTPUT_PUSHPULL);

#ifdef TFT_CSPIN_EN
	Active();
#endif
	Reset();
	InitCommandList(init_cmds1);
	InitCommandList(init_cmds2);
	InitCommandList(init_cmds3);
#ifdef TFT_CSPIN_EN
	Idle();
#endif
}

void TFTSPI_ST7735::set_rotation(uint8_t m) {
	uint8_t madctl = 0;
	uint8_t rotation = m % 4;
	switch (rotation) {
		case 0:
		madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | _format;
			X_Start = TFT_XSTART;
			Y_Start = TFT_YSTART;
			width = TFT_WIDTH;
			height = TFT_HEIGHT;
		break;
		case 1:
		madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | _format;
			X_Start = TFT_YSTART;
			Y_Start = TFT_XSTART;
			width = TFT_HEIGHT;
			height = TFT_WIDTH;
		break;
		case 2:
		madctl = _format;
			X_Start = TFT_XSTART;
			Y_Start = TFT_YSTART/2;
			width = TFT_WIDTH;
			height = TFT_HEIGHT;
		break;
		case 3:
		madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | _format;
			X_Start = TFT_YSTART/2;
			Y_Start = TFT_XSTART;
			width = TFT_HEIGHT;
			height = TFT_WIDTH;
		break;
	}
#ifdef TFT_CSPIN_EN
	Active();
#endif
	WriteCommand(ST7735_MADCTL);
	WriteData(&madctl, 1);
#ifdef TFT_CSPIN_EN
	Idle();
#endif
}

void TFTSPI_ST7735::set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	// column address set
	WriteCommand(ST7735_CASET);
	uint8_t data[4] = {0x00, (uint8_t)(x0 + X_Start), 0x00, (uint8_t)(x1 + X_Start)};
	WriteData(data, sizeof(data));
	// row address set
	WriteCommand(ST7735_RASET);
	data[1] = y0 + Y_Start;
	data[3] = y1 + Y_Start;
	WriteData(data, sizeof(data));
	// write to RAM
	WriteCommand(ST7735_RAMWR);
}

void TFTSPI_ST7735::draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
	if((x >= width) || (y >= height))
		return;
#ifdef TFT_CSPIN_EN
	Active();
#endif
	set_window(x, y, x+1, y+1);
	uint8_t data[2] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};
	WriteData(data, sizeof(data));
#ifdef TFT_CSPIN_EN
	Idle();
#endif
}

void TFTSPI_ST7735::write_color(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *Data, uint16_t size){
#ifdef TFT_CSPIN_EN
	Active();
#endif
	set_window(x1, y1, x2, y2);
	EnableWrite();

	_spi -> transmit_start_dma((uint32_t)Data, size);
	while(!dma_tft_flag);
	_spi -> transmit_stop_dma();
	dma_tft_flag = 0;
#ifdef TFT_CSPIN_EN
	Idle();
#endif
}

void TFTSPI_ST7735::fill_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	if((x >= width) || (y >= height)) return;
	if((x + w - 1) >= width) w = width - x;
	if((y + h - 1) >= height) h = height - y;
	if(w == 0 || h ==0) return;

	uint8_t *disp_buf;
	uint16_t Area = w*h; // Diện tích phần cần tô.
	uint8_t raito = Area/2048; // Tỉ lệ, số lần thực hiện spi transmit dma.
	uint16_t disp_buf_size = 2048;
	if(raito == 0) disp_buf_size = Area; // Nếu kích thước phần cần tô bé hơn 2048 thì cho kích thước buf = Area.

	disp_buf = (uint8_t *)malloc(disp_buf_size*2 * sizeof(uint8_t)); // Cấp phát gấp đôi.
	for(int i=0; i<disp_buf_size; i++){
		disp_buf[i*2]   = (uint8_t)(color >> 8);
		disp_buf[i*2+1] = (uint8_t)(color & 0xFF);
	}
	dma_tft_cnt = raito + 1;
	write_color(x, y, x+w-1, y+h-1, disp_buf, disp_buf_size*2);
	free(disp_buf);
}

void TFTSPI_ST7735::fill_screen(uint16_t color) {
	uint8_t *disp_buf;
	disp_buf = (uint8_t *)malloc(32*128*2 * sizeof(uint8_t));
	for(int i=0; i<32*128; i++){
		disp_buf[i*2]   = (uint8_t)(color >> 8);
		disp_buf[i*2+1] = (uint8_t)(color & 0xFF);
	}
	dma_tft_cnt = 5;
	write_color(0, 0, width-1, height-1, disp_buf, 32*128*2);
	free(disp_buf);
}

void TFTSPI_ST7735::draw_line(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t color) {
	int16_t dX = X2-X1;
	int16_t dY = Y2-Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
//		if (Y2 > Y1) ST7735_VLine(X1, Y1, Y2, color); else ST7735_VLine(X1, Y2, Y1, color);
		if (Y2 > Y1) fill_rectangle(X1, Y1, X1, Y2, color); else fill_rectangle(X1, Y2, X1, Y1, color);
		return;
	}
	if (dY == 0) {
//		if (X2>X1) ST7735_HLine(X1, X2, Y1, color); else ST7735_HLine(X2, X1, Y1, color);
		if (X2 > X1) fill_rectangle(X1, Y1, X2, Y1, color); else fill_rectangle(X2, Y1, X1, Y1, color);
		return;
	}

	dX *= dXsym;
	dY *= dYsym;
	int16_t dX2 = dX << 1;
	int16_t dY2 = dY << 1;
	int16_t di;

	if (dX >= dY) {
		di = dY2 - dX;
		while (X1 != X2) {
			draw_pixel(X1, Y1, color);
			X1 += dXsym;
			if (di < 0) {
				di += dY2;
			} else {
				di += dY2 - dX2;
				Y1 += dYsym;
			}
		}
	} else {
		di = dX2 - dY;
		while (Y1 != Y2) {
			draw_pixel(X1,Y1,color);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	draw_pixel(X1,Y1,color);
}

void TFTSPI_ST7735::draw_rgb_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* Bitmap) {
	if((x >= width) || (y >= height)) return;
	if((x + w - 1) >= width) w = width - x;
	if((y + h - 1) >= height) h = height - y;
	if(w == 0 || h ==0) return;

	uint8_t *disp_buf;
	uint16_t Area = w*h; // Diện tích tấm hình.
	uint8_t raito = Area/2000; // Hệ số chia bitmap theoc chiều dọc.
	uint16_t disp_buf_size = 0;
	uint16_t remainder = 0;
	if(raito == 0) disp_buf_size = Area; // Nếu kích thước phần cần tô bé hơn 2000 thì cho kích thước buf = Area.
	else{
		disp_buf_size = (h/raito) * w;
		remainder = (h - (h/raito)*raito) * w;
	}

	if(raito == 0 && remainder == 0){
		disp_buf = (uint8_t *)malloc(disp_buf_size*2 * sizeof(uint8_t));
		for(uint16_t i=0; i<disp_buf_size; i++){
			disp_buf[i*2]   = (uint8_t)(Bitmap[i] >> 8);
			disp_buf[i*2+1] = (uint8_t)(Bitmap[i] & 0xFF);
		}
		dma_tft_cnt = 1;
		write_color(x, y, x+w-1, y+h-1, disp_buf, disp_buf_size*2);
		free(disp_buf);
	}
	else{
		for(uint8_t numwrite=0; numwrite<raito; numwrite++){
			disp_buf = (uint8_t *)malloc(disp_buf_size*2 * sizeof(uint8_t));
			for(uint16_t i=0; i<disp_buf_size; i++){
				disp_buf[i*2]   = (uint8_t)(Bitmap[i] >> 8);
				disp_buf[i*2+1] = (uint8_t)(Bitmap[i] & 0xFF);
			}
			dma_tft_cnt = 1;
			write_color(x, y + (numwrite*(h/raito)), x+w-1, y+(h/raito)-1, disp_buf, (disp_buf_size-1)*2);
			free(disp_buf);
			Bitmap += disp_buf_size;
		}

		if(remainder != 0){
			disp_buf = (uint8_t *)malloc(remainder*2 * sizeof(uint8_t));
			for(uint16_t i=0; i<remainder; i++){
				disp_buf[i*2]   = (uint8_t)(Bitmap[i] >> 8);
				disp_buf[i*2+1] = (uint8_t)(Bitmap[i] & 0xFF);
			}
			dma_tft_cnt = 1;
			write_color(x, y + ((h/raito)*raito), x+w-1, y+h-1, disp_buf, disp_buf_size*2);
			free(disp_buf);
		}
	}

}

void TFTSPI_ST7735::draw_MkE_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t Color, uint16_t B_Color, const uint8_t* Bitmap){
	uint8_t *disp_buf;
	uint16_t disp_buf_size = sizeof(uint8_t) * 2*w*h;
	disp_buf = (uint8_t *)malloc(disp_buf_size);
	uint8_t num_byte = 0;
	(h%8 != 0)? (num_byte = h/8 + 1) : (num_byte = h/8);
	for(uint8_t X=0; X<w; X++){
		for(uint8_t Y_Byte=0; Y_Byte<num_byte; Y_Byte++){
			for(uint8_t i=0; i<8; i++){
				uint16_t MSB = i*w*2 + Y_Byte*8*w*2 + X*2;
				if(i + Y_Byte*8 >= h) break;

				if(Bitmap[Y_Byte + X*num_byte] & (1<<i)){
					disp_buf[MSB] = (uint8_t)(Color>>8);
					disp_buf[MSB + 1] = Color&0xFF;
				}
				else{ // Pixel sáng màu B_color;
					disp_buf[MSB] = (uint8_t)(B_Color>>8);
					disp_buf[MSB + 1] = B_Color&0xFF;
				}
			}
		}
	}
	dma_tft_cnt = 1;
	write_color(x, y, x+w-1, y+h-1, disp_buf, disp_buf_size);

	free(disp_buf);
}

#ifdef USE_EX_FLASH
void TFTSPI_ST7735::WriteRGBBitmapToFlash(SPIFLASH Flash, uint32_t Write_Address, uint16_t w, uint16_t h, const uint16_t *Bitmap){
	uint32_t writeaddr = Write_Address + 4U;
	uint32_t Area = w*h*2;
	uint16_t num_page = Area / flash_properties.PageSize;
	uint8_t remain_byte = Area % flash_properties.PageSize;
	uint8_t size_buf[4] = {
		(uint8_t)(w >> 8),
		(uint8_t)(w & 0xFF),
		(uint8_t)(h >> 8),
		(uint8_t)(h & 0xFF),
	};
	Flash.WriteBytes(Write_Address, size_buf, 4);

	if(num_page != 0){
		uint8_t *mem_buf = (uint8_t *)malloc(flash_properties.PageSize * sizeof(uint8_t));
		for(uint16_t page=0; page<(flash_properties.PageSize/2); page++){
			for(uint16_t i=0; i<flash_properties.PageSize; i++){
				mem_buf[i*2]   = (uint8_t)(Bitmap[i] >> 8);
				mem_buf[i*2+1] = (uint8_t)(Bitmap[i] & 0xFF);
			}
			Flash.WriteBytes(writeaddr, mem_buf, flash_properties.PageSize);
			writeaddr += flash_properties.PageSize;
			Bitmap += flash_properties.PageSize/2;
		}
		free(mem_buf);
	}
	if(remain_byte != 0){
		uint8_t *mem_buf = (uint8_t *)malloc(remain_byte * sizeof(uint8_t));
		for(uint16_t page=0; page<remain_byte/2; page++){
			for(uint16_t i=0; i<Area/2; i++){
				mem_buf[i*2]   = (uint8_t)(Bitmap[i] >> 8);
				mem_buf[i*2+1] = (uint8_t)(Bitmap[i] & 0xFF);
			}
			Flash.WriteBytes(writeaddr, mem_buf, remain_byte);
		}
		free(mem_buf);
	}
}

void TFTSPI_ST7735::DrawRGBBitmapInFlash(SPIFLASH Flash, uint32_t Read_Address, uint16_t x, uint16_t y){

}

#endif

uint8_t TFTSPI_ST7735::UTF8_GetAddr(char *utf8_char, uint8_t *char_offset){
	*char_offset = 1;
	if((unsigned char)(*utf8_char) < UTF8_start2){ //Ký tự trong bảng mã ascii.
		return (*utf8_char);
	}
	else{ // Ký tự tiếng việt.
		uint32_t utf8_value = 0;
		uint8_t temp = 0xF0 & (*utf8_char);
		if(temp == 0xC0){ //UTF-8 2 byte
			*char_offset = 2;
			utf8_value  = (uint32_t)((unsigned char)*(utf8_char) << 8);
			utf8_value |= (uint32_t)((unsigned char)*(utf8_char+1));
		}
		else if(temp == 0xE0){ //UTF-8 3 byte
			*char_offset = 3;
			utf8_value  = (uint32_t)((unsigned char)*(utf8_char) << 16);
			utf8_value |= (uint32_t)((unsigned char)*(utf8_char+1) << 8);
			utf8_value |= (uint32_t)((unsigned char)*(utf8_char+2));
		}
		else if(temp == 0xF0){ //UTF-8 4 byte
			*char_offset = 4;
			utf8_value  = (uint32_t)((unsigned char)*(utf8_char) << 24);
			utf8_value |= (uint32_t)((unsigned char)*(utf8_char+1) << 16);
			utf8_value |= (uint32_t)((unsigned char)*(utf8_char+2) << 8);
			utf8_value |= (uint32_t)((unsigned char)*(utf8_char+3));
		}
		for(uint8_t i=0; i<sizeof(UTF8_table)/4; i++){
			if(utf8_value == UTF8_table[i]){
				if(i < UTF8_end1) return i + UTF8_start1;
				else return i + UTF8_start2 - UTF8_end1;
			}
		}
	}
	return -1;
}

uint8_t TFTSPI_ST7735::draw_char(uint16_t x, uint16_t y, Font Font, uint16_t Color, uint16_t B_Color, char *Char){
	uint8_t *disp_buf;
	uint8_t  char_offset;
	uint16_t char_addr = (uint16_t)(Font.byte*Font.w * UTF8_GetAddr(Char, &char_offset));
	uint16_t disp_buf_size = sizeof(uint8_t) * 2*Font.w*Font.h;
	disp_buf = (uint8_t *)malloc(disp_buf_size);
	for(uint8_t X=0; X<Font.w; X++){                       // Biến X chạy theo chiều ngang bề rộng font chữ.
		for(uint8_t Y_Byte=0; Y_Byte<Font.byte; Y_Byte++){ // Biến Y_Byte chạy theo chiều dọc số lượng byte ký tự trên 1 cột.
			for(uint8_t i=0; i<8; i++){                    // Biến i chạy lần lượt các bit.
				uint16_t MSB = i*Font.w*2 + Y_Byte*8*Font.w*2 + X*2;
				if(i + Y_Byte*8 >= Font.h) break;

				if(Font.bitmap[char_addr + Y_Byte + X*Font.byte] & (1<<i)){ // Pixel sáng màu color;
					disp_buf[MSB] = (uint8_t)(Color>>8);
					disp_buf[MSB + 1] = Color&0xFF;
				}
				else{ // Pixel sáng màu B_color;
					disp_buf[MSB] = (uint8_t)(B_Color>>8);
					disp_buf[MSB + 1] = B_Color&0xFF;
				}
			}
		}
	}

	dma_tft_cnt = 1;
	write_color(x, y, x+Font.w-1, y+Font.h-1, disp_buf, disp_buf_size);

	free(disp_buf);
	return char_offset;
}

void TFTSPI_ST7735::print(uint16_t x, uint16_t y, Font Font, uint16_t Color, uint16_t B_Color, char *str){
	uint8_t offset = 0;
	uint16_t dx = x;
	while(*str){
		offset = draw_char(dx, y, Font, Color, B_Color, str);
		dx += Font.w;
		str += offset;
	}
}

uint16_t TFTSPI_ST7735::RGB565(uint8_t R, uint8_t G, uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}


void TFTSPI_DMA_Handler(dma_event_t event, void *){
	if(event == DMA_EVENT_TRANFER_COMPLETE){
		dma_tft_cnt--;
		if(dma_tft_cnt == 0){
			dma_tft_cnt = 1;
			dma_tft_flag = 1;
		}
	}
}

