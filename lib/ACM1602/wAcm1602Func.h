//
// I2C制御LCD共通関数 ACM1602 専用
//
//    12/04/24 T.Nakao
//

#ifndef wAcm1602Func_h
#define wAcm1602Func_h


#include <inttypes.h>
#include <Arduino.h>
#include <Wire.h>

#define NUM_LINES 2

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00



class wLcdFunc
{
public:
	wLcdFunc();

	void init(void);

	void clear(void);
	void home(void);
	void setCursor(uint8_t col, uint8_t row);

	void noDisplay(void);
	void display(void);
	void noBlink(void);
	void blink(void);
	void noCursor(void);
	void cursor(void);
	void scrollDisplayLeft(void);
	void scrollDisplayRight(void);
	void leftToRight(void);
	void rightToLeft(void);
	void autoscroll(void);
	void noAutoscroll(void);

	void createChar(uint8_t, uint8_t[] );
	void print(const char *str);
	void print(String string);

	void writeCmd(uint8_t cmd);
	void writeData(uint8_t dat);

	void backlight() {}
	void noBacklight() {}


private:
	uint8_t DisplayMode;
	uint8_t DisplayControl;
};

#endif
