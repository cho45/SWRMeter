//
// I2C制御LCD共通関数 ACM1602 専用
//
//    12/04/24 T.Nakao
//

#include "wAcm1602Func.h"


// コンストラクタ
wLcdFunc::wLcdFunc() {

}

void wLcdFunc::writeCmd(uint8_t cmd) {
	uint8_t ccc;

	Wire.beginTransmission(0x50);

	ccc = 0x00;
	Wire.write(ccc);
	Wire.write(cmd);
	Wire.endTransmission();
}

void wLcdFunc::writeData(uint8_t dat) {
	Wire.beginTransmission(0x50);
	Wire.write(0x80);
	Wire.write(dat);
	Wire.endTransmission();
}


void wLcdFunc::init(void) {
  // LCD初期化
	delay(15);
	writeCmd(0x01);
	delay(5);
	writeCmd(0x38);
	delay(5);
	writeCmd(0x0f);
	delay(5);
	writeCmd(0x06);
	delay(5);

	DisplayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	DisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
}


void wLcdFunc::clear(void)
{
	writeCmd(LCD_CLEARDISPLAY);
	delayMicroseconds(2000);
}

void wLcdFunc::home(void)
{
	writeCmd(LCD_RETURNHOME);
	delayMicroseconds(2000);
}


void wLcdFunc::setCursor(uint8_t col, uint8_t row)
{
	int row_offsets[] = { 0x00, 0x40 };
	if ( row >= NUM_LINES ) {
		row = NUM_LINES - 1;
	}

	writeCmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}


void wLcdFunc::noDisplay() {
	DisplayControl &= ~LCD_DISPLAYON;
	writeCmd(LCD_DISPLAYCONTROL | DisplayControl);
}
void wLcdFunc::display() {
	DisplayControl |= LCD_DISPLAYON;
	writeCmd(LCD_DISPLAYCONTROL | DisplayControl);
}


void wLcdFunc::noCursor() {
	DisplayControl &= ~LCD_CURSORON;
	writeCmd(LCD_DISPLAYCONTROL | DisplayControl);
}
void wLcdFunc::cursor() {
	DisplayControl |= LCD_CURSORON;
	writeCmd(LCD_DISPLAYCONTROL | DisplayControl);
}


void wLcdFunc::noBlink() {
	DisplayControl &= ~LCD_BLINKON;
	writeCmd(LCD_DISPLAYCONTROL | DisplayControl);
}
void wLcdFunc::blink() {
	DisplayControl |= LCD_BLINKON;
	writeCmd(LCD_DISPLAYCONTROL | DisplayControl);
}


void wLcdFunc::scrollDisplayLeft(void) {
	writeCmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void wLcdFunc::scrollDisplayRight(void) {
	writeCmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}



void wLcdFunc::leftToRight(void) {
	DisplayMode |= LCD_ENTRYLEFT;
	writeCmd(LCD_ENTRYMODESET | DisplayMode);
}



void wLcdFunc::rightToLeft(void) {
	DisplayMode &= ~LCD_ENTRYLEFT;
	writeCmd(LCD_ENTRYMODESET | DisplayMode);
}


void wLcdFunc::autoscroll(void) {
	DisplayMode |= LCD_ENTRYSHIFTINCREMENT;
	writeCmd(LCD_ENTRYMODESET | DisplayMode);
}


void wLcdFunc::noAutoscroll(void) {
	DisplayMode &= ~LCD_ENTRYSHIFTINCREMENT;
	writeCmd(LCD_ENTRYMODESET | DisplayMode);
}



void wLcdFunc::createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x07; // we only have 8 locations 0-7

	uint8_t i;
	writeCmd(LCD_SETCGRAMADDR | (location << 3));
	for (i = 0; i < 8; i++) {
		writeData(charmap[i]);
	}
}


void wLcdFunc::print(const char *str) {
	uint8_t i;

	for(i = 0; i < 16; i++) {
		if(str[i] == 0x00) {
			break;
		} else {
			writeData(str[i]);
		}
	}
}

void wLcdFunc::print(String string) {
	print(string.c_str());
}
