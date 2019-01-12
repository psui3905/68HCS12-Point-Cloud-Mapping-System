                                                      
//On Dragon12+ board, the rows and columns of 4x4 keypad are connected to PORTA. 
//As you press any key the value for the key is placed on the 7-segments display of PORTB & LCD display



#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "LCD_KEY.h"
#define  keypad     PORTA
#define  keypad_dir DDRA
#define LCD_DATA PORTK
#define LCD_CTRL PORTK
#define RS 0x01
#define EN 0x02
void msDelay(unsigned int);
int getkey (void);
void LCD_init(void); 
void COMWRT4(unsigned char);
void DATWRT4(unsigned char);

int LCD_welcome(void);

int Scan_Azimuth(void);
int temp;
//int tempnum;
int row,col;




const int keycode[4][4] =
{
1,2,3,10,
4,5,6,11,
7,8,9,12,
'-',0,'#',13
};




/********************Keypad************************/
 // read keypad, return a number 
int getkey(void) {
   
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
         keypad |= 0x01;                   //COLUMN 0 SET HIGH
         row = keypad & 0xF0;              //READ ROWS
         if(row != 0x00){                 //KEY IS IN COLUMN 0
            col = 0;
            break;                        //BREAK OUT OF while(1)
         }
         keypad &= 0xF0;                   //CLEAR COLUMN
         keypad |= 0x02;                   //COLUMN 1 SET HIGH
         row = keypad & 0xF0;              //READ ROWS
         if(row != 0x00){                 //KEY IS IN COLUMN 1
            col = 1;
            break;                        //BREAK OUT OF while(1)
         }

         keypad &= 0xF0;                   //CLEAR COLUMN
         keypad |= 0x04;                   //COLUMN 2 SET HIGH
         row = keypad & 0xF0;              //READ ROWS
         if(row != 0x00){                 //KEY IS IN COLUMN 2
            col = 2;
            break;                        //BREAK OUT OF while(1)
         }
         keypad &= 0xF0;                   //CLEAR COLUMN
         keypad |= 0x08;                   //COLUMN 3 SET HIGH
         row = keypad & 0xF0;              //READ ROWS
         if(row != 0x00){                 //KEY IS IN COLUMN 3
            col = 3;
            break;                        //BREAK OUT OF while(1)
         }
         row = 0;                         //KEY NOT FOUND
      break;                              //step out of while(1) loop to not get stuck
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
                                      //CLOSE WHILE(1)
}                                         //CLOSE MAIN


/********************LCD************************/


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
  msDelay(5);
  LCD_CTRL = LCD_CTRL & ~EN;         //Drop enable to capture command
  msDelay(15);                       //wait
        
  x = (command & 0x0F)<< 2;          // shift low nibble to center of byte for Pk5-Pk2
  LCD_DATA =LCD_DATA & ~0x3C;         //clear bits Pk5-Pk2
  LCD_DATA =LCD_DATA | x;             //send low nibble to PORTK
  LCD_CTRL = LCD_CTRL | EN;          //rais enable
  msDelay(5);
  LCD_CTRL = LCD_CTRL & ~EN;         //drop enable to capture command
  msDelay(15);

}


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
  msDelay(5);
       
  x = (data & 0x0F)<< 2;
  LCD_DATA =LCD_DATA & ~0x3C;                     
  LCD_DATA = LCD_DATA | x;
  LCD_CTRL = LCD_CTRL | EN;
  msDelay(1);
  LCD_CTRL = LCD_CTRL & ~EN;
  msDelay(15);

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

//function to print on the second position of second line on lcd
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
  msDelay(1);
  COMWRT4(0x80);  //set start posistion, home position (first line)
  msDelay(1);
  i_lcd = 0;
  //print the string on lcd
  while (str[i_lcd] != 0x00) {
            
    DATWRT4(str[i_lcd++]);
    msDelay (20);
       
  }
  
}



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

//function to print input degrees on the second line of lcd
void degvalue(int deg){ 
  
  COMWRT4(0xC0);  //set start posistion, home position (second line)
  msDelay(1);    
   
  //get each digit of the input for three cases
  if (deg >= 100){  
    units = (deg%100)%10;
    tenths = (deg/10)/10;
    hundreds = deg/100;
  } 
  else if (deg >= 10){
    units = deg%10;
    tenths = deg/10;
    hundreds = 0;
  } 
  else{  
    units = deg;
    tenths = 0;
    hundreds = 0;
  }
  
  //find the corresponding ascii value of the number, store in a char
  unitdis = units + '0';
  tenthdis = tenths + '0';             
  hunddis = hundreds + '0';
  
  
  //define each char in the input degree string
  str_deg[0] = hunddis;
  str_deg[1] = tenthdis;
  str_deg[2] = unitdis;
  str_deg[3] = 0x00;
  
  //print input on LCD
  i_lcd = 0;
  while (str_deg[i_lcd] != 0x00) {       
    DATWRT4(str_deg[i_lcd++]);
    msDelay (20); 
  }      
 
  
}

//function to print the degrees read from the sensors
void lcddisp(int acc, int mag, int gyro_hea, int gyro_ele){
    
    //start of acc disp
    
    //define a string to indicate what is printing on lcd
    str_in = "accdeg:";
    //display the string on lcd
    strdisp(str_in);
    //display the acc degree on lcd
    degvalue(acc);
    msDelay(200);
    
    //end of acc disp
    
    //start of mag disp
    
    //define a string to indicate what is printing on lcd
    str_in = "magdeg:";
    //display the string on lcd
    strdisp(str_in);
    //display the mag degree on lcd
    degvalue(mag);
    msDelay(200);
    
    //end of magn disp
    
    
    //start of gyro disp
    
    //define a string to indicate what is printing on lcd
    str_in = "gyroheadeg:";
    //display the string on lcd
    strdisp(str_in);
    //display the gyro heading degree value on lcd
    degvalue(gyro_hea);
    msDelay(200);
   
    //define a string to indicate what is printing on lcd
    str_in = "gyroeleeg:";
    //display the string on lcd
    strdisp(str_in);
    //display the gyro elevation degree on lcd
    degvalue(gyro_ele);
    msDelay(200);
    
    //end of gyro disp
    
    
    
}

//function to print the degrees of servos
void lcd_servo(int servo_hea, int servo_ele){
   
    //start of servo disp
    
    //define a string to indicate what is printing on lcd
    str_in = "servoheadeg:";
    //display the string on lcd
    strdisp(str_in);
    //display the servo heading degree value on lcd
    degvalue(servo_hea);
    msDelay(200);
   
    //define a string to indicate what is printing on lcd
    str_in = "servoeledeg:";
    //display the string on lcd
    strdisp(str_in);
    //display the servo elevation degree on lcd
    degvalue(servo_ele);
    msDelay(200);
    
    //end of servo disp
    
    
    
}


//function to print input degrees on the second line of lcd
void lcd_area(int area){ 
  
  COMWRT4(0x80);  //set start posistion, home position (first line)
  msDelay(1);    
   
  //get each digit of the input for four cases
  if (area >= 1000){
  	units = (area%100)%10;
    tenths = (area/10)%10;
    hundreds = (area/100)%10;
    thousands = (area/100)/10;  
    
  } 
  else if (area >= 100){
    units = (area%100)%10;
    tenths = (area/10)/10;
    hundreds = area/100;
    thousands = 0;
  } 
  else if (area >= 10){  
    units = area%10;
    tenths = area/10;
    hundreds = 0;
    thousands = 0;
  }
  else{
  	units = area;
    tenths = 0;
    hundreds = 0;
    thousands = 0;
  }
  
  //find the corresponding ascii value of the number, store in a char
  unitdis = units + '0';
  tenthdis = tenths + '0';
  hunddis = hundreds + '0';
  thoudis = thousands + '0';
  
  //define each char in the input degree string
  str_area[0] = thoudis;
  str_area[1] = hunddis;
  str_area[2] = tenthdis;
  str_area[3] = unitdis;
  str_area[4] = 0x00;
  
  //print input on LCD
  i_lcd = 0;
  while (str_area[i_lcd] != 0x00) {       
    DATWRT4(str_area[i_lcd++]);
    msDelay (20); 
  }      
 
  
}

// return mode number 
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


// return Azimuth 
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
  
  





void msDelay(unsigned int itime){
unsigned int i; unsigned int j;
   for(i=0;i<itime;i++)
      for(j=0;j<4000;j++);
}