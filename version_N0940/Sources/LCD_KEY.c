/*
 ***************************************************************
 * The University of Sydney                                      *
 * MTRX2700  Mechatronics 2                                      *
 *                                                               *
 * Major Project - Mapping System                                *
 *                                                               *
 * Group 9 Tuesday Lab                                           *
 * Authors:                                                      *
 *     - Pengwei Sui (Charles)                                   *
 *     - Hongzhan Yu (Tom)                                       *
 *     - Qianbi Yu (Queenie)                                     *
 *     - Haijun Zeng (Clarence)                                  *
 *                                                               *
 * Project Start Date: 08/05/2018                                *
 * Project End Date:   05/06/2018                                *
 *                                                               *
 * LCD_KEY function:                                             *
 *   This program includes a keypad input function - as the user *
 *   press any key, the value for the key is return from         *
 *   the function.                                               *
 *   There are also serval LCD functions which are used to       *
 *   1. ask for user inputs and display the value from the       *
 *   keypad                                                      *
 *   2. display the Heading & Elevation reading from three       *
 *   sensors and servos                                          *
 ***************************************************************

 VERSION HISTORY:
 --------------------------------------------------------------------------------------------
 |   DATE     |  TIME  |   AUTHOR       | VERSION | COMMENTS                                |
 --------------------------------------------------------------------------------------------
 | 21/05/2018 |12:00pm | Queenie        | v0.1    | Keypad input function & LCD scan        |
 | 24/05/2018 | 2:00pm | Tom            | v0.2    | Error handle                            |
 | 28/05/2018 | 4:15pm | Clarence       | v1.0    | Working on LCD display sensor reading   |
 | 30/05/2018 | 5:00pm | Tom            | v1.1    | Error handle                            |
 | 05/06/2018 | 3:30pm | Group          | v2.0    | Submission                              |
 
 */



#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "LCD_KEY.h"
#define  keypad     PORTA
#define  keypad_dir DDRA
#define LCD_DATA PORTK
#define LCD_CTRL PORTK
#define RS 0x01
#define EN 0x02

// Declaration of the functions
//Keypad Input Function
int getkey (void);

//LCD data writing functions
void COMWRT4(unsigned char);
void DATWRT4(unsigned char);

//Delay function
void msDelay(unsigned int);

//Functions to display the reading from gyros

void gyro_heading_lcd_sentense(void);
void gyro_heading_lcd_data(int gx);

void gyro_elevation_lcd_sentense(void);
void gyro_elevation_lcd_data(int gy);

//Functions to display the reading from servos

void servo_elevation_lcd_sentense(void);
void servo_elevation_lcd_data(int servo_e);

void servo_heading_lcd_sentense(void);
void servo_heading_lcd_data(int servo_h);

//Functions to display the reading from accelomter

void accel_elevation_lcd_sentense(void);
void accel_elevation_lcd_data(int accel_e);

//Keypad input function variable declaration
int temp;
int row,col;

// The 4x4 keypad lookup table
// 0-9 key indicate value of 0-9 , A-D key indicate value of 10-13
//* key indicate nagative sign -

const int keycode[4][4] =
{
1,2,3,10,
4,5,6,11,
7,8,9,12,
'-',0,'#',13
};




/****************************************** Keypad ********************************************/
 // read keypad, return a number 
 
int getkey(void) {
   char a=0x01;
   char colnum;
   while(1){                              //OPEN WHILE(1)
      do{                                 //OPEN do1
         keypad = keypad | 0x0F;            //COLUMNS SET HIGH
         row = keypad & 0xF0;              //READ ROWS
      }while(row == 0x00);                //WAIT UNTIL KEY PRESSED //CLOSE do1



      do{                                 //OPEN do2
         do{                              //OPEN do3
            msDelay(1);                   //WAIT
            row = keypad & 0xF0;           //READ ROWS
         }while(row == 0x00);             //CHECK FOR KEY PRESS //CLOSE do3
         
         msDelay(15);                     //WAIT FOR DEBOUNCE
         row = keypad & 0xF0;
      }while(row == 0x00);                //FALSE KEY PRESS //CLOSE do2

      while(1){                           //OPEN while(1)
         keypad &= 0xF0;                   //CLEAR COLUMN
         keypad |= a;                   //COLUMN 0 SET HIGH
         row = keypad & 0xF0;              //READ ROWS
         if(row != 0x00){                 //KEY IS IN COLUMN 0
            col = colnum;
            break;                        //BREAK OUT OF while(1)
         }
         a<<1;
         colnum+=1;
      }                                   //end while(1)

      if(row == 0x10){
         temp=keycode[0][col];
         //PORTB=keycode[0][col];         //OUTPUT TO PORTB LED
 
      }
      else if(row == 0x20){
         temp=keycode[1][col];
         //PORTB=keycode[1][col];
 
      }
      else if(row == 0x40){
         temp=keycode[2][col];
         //PORTB=keycode[2][col];
         
      }
      else if(row == 0x80){
         temp=keycode[3][col];
         //PORTB=keycode[3][col];
         
      }
      

      do{
         msDelay(15);
         keypad = keypad | 0x0F;            //COLUMNS SET HIGH
         row = keypad & 0xF0;              //READ ROWS
      }while(row != 0x00);                //MAKE SURE BUTTON IS NOT STILL HELD
   
      return temp;
   }
}
                                      //CLOSE WHILE(1)
                                        //CLOSE MAIN


/********************************** Liquid Crystal Display (LCD) *********************************/

//LCD function variable declaration


#define LCD_DATA PORTK
#define LCD_CTRL PORTK
#define RS 0x01
#define EN 0x02



//function variable declaration
int i_lcd;
int units;
int tenths;
int hundreds;
int thousands;
char unitdis;
char tenthdis;
char hunddis;
char thoudis;
char str_deg[4];
char str_area[5];
char str_dis[4];
char *str_in;


int i_lcd;

//LCD Initial Function
void openLCD(void) 
{

   DDRK = 0xFF;   
   COMWRT4(0x33);   //reset sequence provided by data sheet
   msDelay(1);
   COMWRT4(0x32);   //reset sequence provided by data sheet
   msDelay(1);
   COMWRT4(0x28);   //Function set to four bit data length
                                         //2 line, 5 x 7 dot format
   msDelay(1);
   COMWRT4(0x06);  //entry mode set, increment, no shift
   msDelay(1);
   COMWRT4(0x0F);  //Display set, disp on, cursor on, blink
   msDelay(1);
   COMWRT4(0x01);  //Clear display
   msDelay(1);
   

}

//LCD data writing function
// Reference from Eduardo Nebot
void COMWRT4(unsigned char command)
{
  unsigned char x;
        
  x = (command & 0xF0) >> 2;         //shift high nibble to center of byte for Pk5-Pk2
  LCD_DATA =LCD_DATA & ~0x3C;          //clear bits Pk5-Pk2
  LCD_DATA = LCD_DATA | x;          //sends high nibble to PORTK
  msDelay(1);
  LCD_CTRL = LCD_CTRL & ~RS;         //set RS to command (RS=0)
  msDelay(1);
  LCD_CTRL = LCD_CTRL | EN;          //rais enable
  msDelay(3);
  LCD_CTRL = LCD_CTRL & ~EN;         //Drop enable to capture command
  msDelay(5);                       //wait
        
  x = (command & 0x0F)<< 2;          // shift low nibble to center of byte for Pk5-Pk2
  LCD_DATA =LCD_DATA & ~0x3C;         //clear bits Pk5-Pk2
  LCD_DATA =LCD_DATA | x;             //send low nibble to PORTK
  LCD_CTRL = LCD_CTRL | EN;          //rais enable
  msDelay(3);
  LCD_CTRL = LCD_CTRL & ~EN;         //drop enable to capture command
  msDelay(5);

}

//LCD data writing function
// Reference from Eduardo Nebot
void DATWRT4(unsigned char data)
{
  unsigned char x;
       
  x = (data & 0xF0) >> 2;
  LCD_DATA =LCD_DATA & ~0x3C;                     
  LCD_DATA = LCD_DATA | x;
  msDelay(1);
  LCD_CTRL = LCD_CTRL | RS;
  msDelay(1);
  LCD_CTRL = LCD_CTRL | EN;
  msDelay(1);
  LCD_CTRL = LCD_CTRL & ~EN;
  msDelay(1);
       
  x = (data & 0x0F)<< 2;
  LCD_DATA =LCD_DATA & ~0x3C;                     
  LCD_DATA = LCD_DATA | x;
  LCD_CTRL = LCD_CTRL | EN;
  msDelay(1);
  LCD_CTRL = LCD_CTRL & ~EN;
  msDelay(5);

}


//function to print on the first position of second line on lcd
void lcd_digit1(int digit1){
  digit1 += '0';  //transfer the input to ascii number	
  COMWRT4(0xC0);  //set start posistion, home position
  msDelay(1);
  DATWRT4(digit1);  
}

//function to print on the second position of second line on lcd
void lcd_digit2(int digit2){
  digit2 += '0';
  COMWRT4(0xC1);  //set start posistion, home position
  msDelay(1);

  DATWRT4(digit2); 
}

//function to print on the third position of Third line on lcd
void lcd_digit3(int digit3){
  digit3 += '0';
  COMWRT4(0xC2);  //set start posistion, home position
  msDelay(1);

  DATWRT4(digit3); 
}


//function to display a string on the first line on lcd
//clear the lcd first and then display the input string
void strdisp(char* str){
  COMWRT4(0x01);	//clear the lcd
 // msDelay(1);
  COMWRT4(0x80);  //set start posistion, home position (first line)
  //msDelay(1);
  i_lcd = 0;
  //print the string on lcd
  while (str[i_lcd] != 0x00) {      
    DATWRT4(str[i_lcd++]);
    msDelay (5);   
  }
}


//function to display a string on the second line on lcd
//clear the lcd first and then display the input string
void strdisp2(char* str){
  msDelay(1);
  COMWRT4(0xC0);  //set start posistion, home position (first line)
  msDelay(1);
  i_lcd = 0;
  //print the string on lcd
  while (str[i_lcd] != 0x00) {   
    DATWRT4(str[i_lcd++]);
    msDelay (20); 
  }
  
}

// Tracking mode Azimuth & Elevation Configuration 
// Get input from the keypad and return Azimuth degree

int LCD_welcome(void){
    int select_mode = 0;
    strdisp("MTRX2700 Group 9");
    strdisp2("Select Mode:");
    select_mode = getkey();
    while(select_mode!=1&&select_mode!=2&&select_mode!=3){
      strdisp("Error");
      select_mode = getkey();
    }
    return select_mode;
}


// Tracking mode Azimuth & Elevation Configuration 
// Get input from the keypad and return Azimuth degree

int Scan_Azimuth(void){
    int Azi=0;
    int digit0,digit1,digit2;
    
    while(Azi<30||Azi>160){
      
      
      strdisp("Azimuth 30-160");
      digit0= getkey();
      lcd_digit1(digit0);
    

      digit1= getkey();
      lcd_digit2(digit1);
      
      digit2 = getkey();
      lcd_digit3(digit2);

      Azi = digit0 * 100 + digit1  * 10 + digit2;
    }
    return Azi;
}

// Get input from the keypad and return Elevation degree
int Scan_Elevation(void){

    int Ele=-70;
    int digit0,digit1,digit2;
    
    while(Ele<-60||Ele>60){
      strdisp("Elevation -60-60");
      digit0= getkey();
      if(digit0=='-'){	
        COMWRT4(0xC0);  //set start posistion, home position
        msDelay(1);
        DATWRT4(digit0);    
      }else{
      lcd_digit1(digit0);
      }
      
      
      digit1= getkey();
      lcd_digit2(digit1);
      
      digit2 = getkey();
      lcd_digit3(digit2);
      
      if(digit0=='-') {
      Ele = (-1 *(digit1 *10 +digit2 ));
      }else{
      Ele = digit0 * 100 + digit1  * 10 + digit2;
      }
    }
    return Ele;
}



// Mapping Mode Azimuth & Elevation Configuration 
// Get input from the keypad and return Azimuth start degree

int Scan_Azimuth_start(void){
    int Azi=0;
    int digit0,digit1,digit2;
    
    while(Azi<30||Azi>160){
      
      
      strdisp("Azi start 30-160");
      digit0= getkey();
      lcd_digit1(digit0);
    

      digit1= getkey();
      lcd_digit2(digit1);
      
      digit2 = getkey();
      lcd_digit3(digit2);

      Azi = digit0 * 100 + digit1  * 10 + digit2;
    }
    return Azi;
}

// Get input from the keypad and return Azimuth end degree

int Scan_Azimuth_end(void){
    int Azi=0;
    int digit0,digit1,digit2;
    
    while(Azi<30||Azi>160){
      
      
      strdisp("Azi end 30-160");
      digit0= getkey();
      lcd_digit1(digit0);
    

      digit1= getkey();
      lcd_digit2(digit1);
      
      digit2 = getkey();
      lcd_digit3(digit2);

      Azi = digit0 * 100 + digit1  * 10 + digit2;
    }
    return Azi;
}

// Get input from the keypad and return Elevation start degree
int Scan_Elevation_start(void){

    int Ele=-70;
    int digit0,digit1,digit2;
    
    while(Ele<-60||Ele>60){
      strdisp("Ele start -60-60");
      digit0= getkey();
      if(digit0=='-'){	
        COMWRT4(0xC0);  //set start posistion, home position
        msDelay(1);
        DATWRT4(digit0);    
      }else{
      lcd_digit1(digit0);
      }
      
      
      digit1= getkey();
      lcd_digit2(digit1);
      
      digit2 = getkey();
      lcd_digit3(digit2);
      
      if(digit0=='-') {
      Ele = (-1 *(digit1 *10 +digit2 ));
      }else{
      Ele = digit0 * 100 + digit1  * 10 + digit2;
      }
    }
    return Ele;
}

// Get input from the keypad and return Elevation start degree
int Scan_Elevation_end(void){

    int Ele=-70;
    int digit0,digit1,digit2;
    
    while(Ele<-60||Ele>60){
      strdisp("Ele end -60-60");
      digit0= getkey();
      if(digit0=='-'){	
        COMWRT4(0xC0);  //set start posistion, home position
        msDelay(1);
        DATWRT4(digit0);    
      }else{
      lcd_digit1(digit0);
      }
      
      
      digit1= getkey();
      lcd_digit2(digit1);
      
      digit2 = getkey();
      lcd_digit3(digit2);
      
      if(digit0=='-') {
      Ele = (-1 *(digit1 *10 +digit2 ));
      }else{
      Ele = digit0 * 100 + digit1  * 10 + digit2;
      }
    }
    return Ele;
}

// Mapping Mode step change configuration
// Get input from the keypad and return step change
 
void Scan_StepChange(void){
  
  int step;
  
  do{   
    strdisp("Step Change 1-4");
    step = getkey();
    lcd_digit1(step);
    
  }while(step<1||step>4);
  
  //return step;
  
}

// Mapping Mode number of samples per orientation configuration
// Get input from the keypad and return number of samples per orientation
 
void Scan_NumberOfSample(void){
  
  int sampleNum = 1;
  
  do{
    strdisp("NumofSamples 1-3");
    sampleNum = getkey();
    lcd_digit1(sampleNum);
  } while(sampleNum <1||sampleNum >3);
    
    
    
  //return sampleNum;
  
}

// Mapping Mode Sample Frequency configuration
// Get input from the keypad and return  Sample Frequency

void Scan_SampleFrequency(void){
    
    int frequency=0;
    int digit0,digit1;
    
    do{
      strdisp("Frequency 10-50");
      digit0= getkey();
      lcd_digit1(digit0);
    

      digit1= getkey();
      lcd_digit2(digit1);

      frequency = digit0 * 10 + digit1;
    }while(frequency<10||frequency>50);     
    //return frequency;
}

// function to print the heading angle from gyro on the LCD

void gyro_heading_lcd_sentense(void){
  str_in = "Heading - gyro:";
  strdisp(str_in);
}

void gyro_heading_lcd_data(int gx){
  
  COMWRT4(0xC0);  //set start position, home position (second line)
  msDelay(1);
  
  //print the data of gx
  if (gx >= 100){  
    units = gx%10;
    tenths = (gx%100)/10;
    hundreds = gx/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = hunddis;
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }      
  }
   
  else if (gx >= 10){
    units = gx%10;
    tenths = gx/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = tenthdis;
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);    
    }
  }
  
  else if (gx >= 0){  
    units = gx;
    unitdis = units + '0';

    str_deg[0] = unitdis;
    str_deg[1] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    }
  } 
  
  else if(gx <= -100){
    gx = -gx;
  
    units = gx%10;
    tenths = (gx%100)/10;
    hundreds = gx/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = '-';
    str_deg[1] = hunddis;
    str_deg[2] = tenthdis;
    str_deg[3] = unitdis;
    str_deg[4] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }     
  }
  
    else if (gx <= -10){
    gx = -gx;
    
    units = gx%10;
    tenths = gx/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = '-';
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);  
    } 
  } 
  
  else if (gx <= 0){
    gx = -gx;
  
    units = gx;
    unitdis = units + '0';

    str_deg[0] = '-';
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    } 
  }
  DATWRT4(' ');
  DATWRT4(' ');
}

// function to print the elevation angle from gyro on the LCD
void gyro_elevation_lcd_sentense(void){
  str_in = "elevation - gyro";
  strdisp(str_in);                
}
    
void gyro_elevation_lcd_data(int gy){
  
  COMWRT4(0xC0);  //set start position, home position (second line)
  msDelay(1);
   
   
   //print the data of gy
  if (gy >= 100){  
    units = gy%10;
    tenths = (gy%100)/10;
    hundreds = gy/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = hunddis;
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }      
  }
   
  else if (gy >= 10){
    units = gy%10;
    tenths = gy/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = tenthdis;
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);    
    }
  }
  
  else if (gy >= 0){  
    units = gy;
    unitdis = units + '0';

    str_deg[0] = unitdis;
    str_deg[1] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    }
  } 
  
   else if(gy <= -100){
    gy = -gy;
  
    units = gy%10;
    tenths = (gy%100)/10;
    hundreds = gy/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = '-';
    str_deg[1] = hunddis;
    str_deg[2] = tenthdis;
    str_deg[3] = unitdis;
    str_deg[4] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }     
  }
  
  else if (gy <= -10){
    gy = -gy;
    
    units = gy%10;
    tenths = gy/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = '-';
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);  
    } 
  } 
  
  else if (gy <= 0){
    gy = -gy;
  
    units = gy;
    unitdis = units + '0';

    str_deg[0] = '-';
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    } 
  }
  
  DATWRT4(' ');
  DATWRT4(' ');  
}

void servo_elevation_lcd_sentense(void){
  str_in = "elevation-servo";
  strdisp(str_in);
}

void servo_elevation_lcd_data(int servo_e) { 
  COMWRT4(0xC0);  //set start position, home position (second line)
  msDelay(1);   
   
   //print the data of gy
  if (servo_e >= servo_e){  
    units = servo_e%10;
    tenths = (servo_e%100)/10;
    hundreds = servo_e/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = hunddis;
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }      
  }
   
  else if (servo_e >= 10){
    units = servo_e%10;
    tenths = servo_e/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = tenthdis;
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);    
    }
  }
  
  else if (servo_e >= 0){  
    units = servo_e;
    unitdis = units + '0';

    str_deg[0] = unitdis;
    str_deg[1] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    }
  } 
  
   else if(servo_e <= -100){
    servo_e = -servo_e;
  
    units = servo_e%10;
    tenths = (servo_e%100)/10;
    hundreds = servo_e/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = '-';
    str_deg[1] = hunddis;
    str_deg[2] = tenthdis;
    str_deg[3] = unitdis;
    str_deg[4] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }     
  }
  
  else if (servo_e <= -10){
    servo_e = -servo_e;
    
    units = servo_e%10;
    tenths = servo_e/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = '-';
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);  
    } 
  } 
  
  else if (servo_e <= 0){
    servo_e = -servo_e;
  
    units = servo_e;
    unitdis = units + '0';

    str_deg[0] = '-';
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    } 
  }
  
  DATWRT4(' ');
  DATWRT4(' ');   
}

void accel_elevation_lcd_sentense(void){
  str_in = "elevation-accel";
  strdisp(str_in);
}

void accel_elevation_lcd_data(int accel_e) { 
  COMWRT4(0xC0);  //set start position, home position (second line)
  msDelay(1);   
   
   //print the data of gy
  if (accel_e >= accel_e){  
    units = accel_e%10;
    tenths = (accel_e%100)/10;
    hundreds = accel_e/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = hunddis;
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }      
  }
   
  else if (accel_e >= 10){
    units = accel_e%10;
    tenths = accel_e/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = tenthdis;
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);    
    }
  }
  
  else if (accel_e >= 0){  
    units = accel_e;
    unitdis = units + '0';

    str_deg[0] = unitdis;
    str_deg[1] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    }
  } 
  
   else if(accel_e <= -100){
    accel_e = -accel_e;
  
    units = accel_e%10;
    tenths = (accel_e%100)/10;
    hundreds = accel_e/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = '-';
    str_deg[1] = hunddis;
    str_deg[2] = tenthdis;
    str_deg[3] = unitdis;
    str_deg[4] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }     
  }
  
  else if (accel_e <= -10){
    accel_e = -accel_e;
    
    units = accel_e%10;
    tenths = accel_e/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = '-';
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);  
    } 
  } 
  
  else if (accel_e <= 0){
    accel_e = -accel_e;
  
    units = accel_e;
    unitdis = units + '0';

    str_deg[0] = '-';
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    } 
  }
  
  DATWRT4(' ');
  DATWRT4(' ');   
}

void servo_heading_lcd_sentense(void){
  str_in = "heading - servo";
  strdisp(str_in);
}

void servo_heading_lcd_data(int servo_h) { 
  COMWRT4(0xC0);  //set start position, home position (second line)
  msDelay(1);   
   
   //print the data of gy
  if (servo_h >= 100){  
    units = servo_h%10;
    tenths = (servo_h%100)/10;
    hundreds = servo_h/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = hunddis;
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }      
  }
   
  else if (servo_h >= 10){
    units = servo_h%10;
    tenths = servo_h/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = tenthdis;
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);    
    }
  }
  
  else if (servo_h >= 0){  
    units = servo_h;
    unitdis = units + '0';

    str_deg[0] = unitdis;
    str_deg[1] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    }
  } 
  
   else if(servo_h <= -100){
    servo_h = -servo_h;
  
    units = servo_h%10;
    tenths = (servo_h%100)/10;
    hundreds = servo_h/100;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';             
    hunddis = hundreds + '0';
    
    str_deg[0] = '-';
    str_deg[1] = hunddis;
    str_deg[2] = tenthdis;
    str_deg[3] = unitdis;
    str_deg[4] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1); 
    }     
  }
  
  else if (servo_h <= -10){
    servo_h = -servo_h;
    
    units = servo_h%10;
    tenths = servo_h/10;
    
    unitdis = units + '0';
    tenthdis = tenths + '0';
    
    str_deg[0] = '-';
    str_deg[1] = tenthdis;
    str_deg[2] = unitdis;
    str_deg[3] = 0x00;
    
    //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);  
    } 
  } 
  
  else if (servo_h <= 0){
    servo_h = -servo_h;
  
    units = servo_h;
    unitdis = units + '0';

    str_deg[0] = '-';
    str_deg[1] = unitdis;
    str_deg[2] = 0x00;
    
     //print input on LCD
    i_lcd = 0;
    while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (1);   
    } 
  }
  
  DATWRT4(' ');
  DATWRT4(' ');   
}



// Delay function (ms)   

void msDelay(unsigned int itime){
unsigned int i; unsigned int j;
   for(i=0;i<itime;i++)
      for(j=0;j<4000;j++);
}