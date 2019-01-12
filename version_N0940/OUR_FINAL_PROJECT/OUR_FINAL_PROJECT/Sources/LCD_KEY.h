
int getkey(void);

/********************LCD************************/


//lcd function declaration
void openLCD(void);
void COMWRT4(unsigned char);
void DATWRT4(unsigned char);
void lcd_digit1(int digit1);
void lcd_digit2(int digit2);
void lcd_digit3(int digit3);
void strdisp(char* str); 
void degvalue(int deg);
void lcddisp(int acc, int mag, int gyro_hea, int gyro_ele);
void lcd_servo(int servo_hea, int servo_ele);
void lcd_area(int area);
void strdisp2(char* str);
int getkey(void);

int LCD_welcome(void);
int Scan_Azimuth_start(void);
int Scan_Elevation_end(void);
int Scan_Azimuth_end(void);
int Scan_Elevation_start(void);

/**********************SUBROUTINES***********/

void msDelay(unsigned int itime);

