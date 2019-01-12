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
 * Mapping system  :                                             *
 *   This program is used to build an accurate map of a area     *
 *   The final product present the 3D information of the area    *
 *   in the pc with real-time and offline modes ( using MATLAB). *
 *   The program will either read the input from terminal of PC  *
 *   or keypad input from Dragon 12 Board. There are three       *
 *   functional mode - Tracking , Mapping&Plotting, Orientation  *
 *   Demonstration                                               *
 ***************************************************************

 VERSION HISTORY:
 --------------------------------------------------------------------------------------------
 |   DATE     |  TIME  |   AUTHOR       | VERSION | COMMENTS                                |
 --------------------------------------------------------------------------------------------
 | 21/05/2018 |12:00pm | Charles        | v0.1    | Build the PWM function                  |
 | 24/05/2018 | 2:00pm | Tom            | v0.2    | Manipulation of the sensor data         |
 | 28/05/2018 | 4:15pm | Clarence       | v1.0    | Modify the sensor data                  |
 | 30/05/2018 | 5:00pm | Queenie        | v1.1    | Error handle                             |
 | 05/06/2018 | 3:30pm | Group          | v2.0    | Submission                              |
 
 */
 

#include <math.h>
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "iic.h"
#include "pll.h"
#include "sci1.h"
#include <math.h>
#include "LCD_KEY.h"
#include "l3g4200.h"  // register's definitions    ( not used by ed )


volatile uint8_t alarmSignaled1 = 0; /* Flag set when alarm 1 signaled */

volatile uint16_t currentTime1 = 0; /* variables private to timeout routines */
uint16_t alarmTime1 = 0;
volatile uint8_t alarmSet1 = 0;

// void INTERRUPT timer6(void);

void setAlarm1(uint16_t msDelay1);
void delay1(uint16_t msDelay1);
void Init_TIMERS (void);

#define laser_wr  0xc4
#define laser_rd  0xc5

#define gyro_wr 0xD2
#define gyro_rd 0xD3



#define accel_wr 0xA6    //
#define accel_rd 0xA7    //
#define ADXL345_TO_READ 6

#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32
#define ADXL345_DATA_FORMAT 0x31

#define ADXL345_OFSX 0x1E
#define ADXL345_OFSY 0x1F
#define ADXL345_OFSZ 0x20

#define ALPHA 0.5

#define magnet_wr  0x3C
#define magnet_rd  0x3D

#define HM5883_MODE_REG 0x02
#define HM5883_DATAX0 0x03



#define BUFF_SIZE	100

char buff[BUFF_SIZE];
int gxraw[BUFF_SIZE];
int gyraw[BUFF_SIZE],gzraw[BUFF_SIZE];

int axraw[BUFF_SIZE];
int ayraw[BUFF_SIZE],azraw[BUFF_SIZE];

int mxraw[BUFF_SIZE];
int myraw[BUFF_SIZE],mzraw[BUFF_SIZE];


char windows[4];                                 // Corresponds to which display the value is required to be
char count;
char LCD_count;



void segments(void);


void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw);
void accel_init(void);

void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw);
void magnet_init(void);

void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw);
void gyro_init(void);

void data_process(void);

unsigned char LidarWriteAddr = 0xc4;
uint8_t LidarReadAddr = 0xc5;
uint8_t Lidar2ByteRead = 0x8f;

uint16_t Dist;



// !!!!!!!! Charles Added PWM Function
#define PI  3.14159265
int k;
int DIP_change_Different_flag;
int PC_or_board;
int user_command;
int PTH_value;
int DIP_display_No;
int DIP_Model_quit_flag;

void MainInput_moudle_Select(void);
void Help_function(void);
void Commend_configure(void);
int PC_demo_modeSelect(void);
void PWM_init(void);
void PWM_OFF(void);
//void DIP_Switch_init(void);
void PWM_mapping(void);
void PWM_SCI_Tracking(void);
void Orientation_demo_modle(void);
void keypad_mode_angle(void);

void sensor_offset(void);

// Tracking Varaible (Charles):

signed int azimuth_Start_degree;
signed int azimuth_End_degree;

signed int elevation_Start_degree;
signed int elevation_End_degree;

signed int azimuth_Start_DCycle;
signed int azimuth_End_DCycle;

signed int elevation_Start_DCycle;
signed int elevation_End_DCycle;

int selected_mode;
unsigned short resolution;

// !!!!!!!!Hongzhan Yu modified in 24/05/2018
unsigned int overflow;
unsigned int is_rising;
unsigned int edge_rising;
unsigned int edge_falling;
unsigned long distance;
unsigned int rotation;
#define mGperLSB 0.039;
double Ax, Ay, Az;
double theta;
double Gx,Gy,Gz;
double GOX,GOY,GOZ;
double sens;
double dt;
double polar;
double position1,position2;

// Haijun Zeng 7 segment
int thous, hunds, ttens, ddigits;


int tempnum;

int lookupDigits[14];


//int A_Start_degree;
//int A_End_degree;

//int E_Start_degree;
//int E_End_degree;



// ***********************************************************MAIN LOOP HERE*********************************************************************



void main(void) {
    /* put your own code here */

   

   //****** initialization here ******
   // The next 4 lines just to make sure all is working
   // are not needed for final program
   
   DDRB= 0xFF;   /* Port B output */
   DDRJ= 0xFF;   // Port J to Output
   PTJ = 0x00;   // enable LEDs
   PORTB=0x55;     // debuging info


  //  DDRB = 0xFF;        // Enable PORT B
  //  DDRJ= 0xFF;         // Enable PORT J
  //  DDRP= 0xFF;         // Enable PORT P
  //  PTJ=  0x00;
  //  DDRH= 0x00;

   // varibles initial here
  overflow=0;
  is_rising=0;
  edge_rising=0;
  edge_falling=0;
  distance=0;
  rotation=0;
  DIP_display_No = 1;

  // Initialization of the servo and sensors 
  PLL_Init();  // make sure we are runnign at 24 Mhz
  EnableInterrupts;
  // You can connect the serial port, set it to 9600 bauds
  SCI1_Init(BAUD_9600);   // capped at 9600, if PLL inactive (4 MHz bus)
  //SCI1_OutString("Program Starting ");      // should display this
  Init_TIMERS();
  iicinit();
  gyro_init();     // l3g4200 setup
  accel_init();
  magnet_init();
  PWM_init();
  //DIP_Switch_init();
  openLCD();


  /**/

  DDRB = 0xFF;                            //MAKE PORTB OUTPUT
  //DDRJ |=0x02;
  //PTJ &=~0x02;                            //ACTIVATE LED ARRAY ON PORT B
  DDRP |=0xFF;                           //
  PTP |=0x0F;                            //TURN OFF 7SEG LED
  DDRA = 0x0F;                           //MAKE ROWS INPUT AND COLUMNS OUTPUT

  // Lookup table for 7-seg display
  lookupDigits[0] = 0x3F;
  lookupDigits[1] = 0x06;
  lookupDigits[2] = 0x5B;
  lookupDigits[3] = 0x4F;
  lookupDigits[4] = 0x66;
  lookupDigits[5] = 0x6D;
  lookupDigits[6] = 0x7D;
  lookupDigits[7] = 0x07;
  lookupDigits[8] = 0x7F;
  lookupDigits[9] = 0x67;
  lookupDigits[10] = 0x77;  // A
  lookupDigits[11] = 0x7C;  // B
  lookupDigits[12] = 0x58;  // C
  lookupDigits[13] = 0x5E;  // D
  
  windows[0]= 0x0E;
  windows[1]= 0x0D;
  windows[2]= 0x0B;
  windows[3]= 0x07;

  EnableInterrupts;

  //Initialization of the duty cycle for PWM
  azimuth_Start_DCycle = 0;
  azimuth_End_DCycle = 0;
  elevation_Start_DCycle = 0;
  elevation_End_DCycle = 0;
  
  
  //Initialization of Azimuth and Elevation degree
  azimuth_Start_degree = 0;
  azimuth_End_degree = 0;
  elevation_Start_degree = 0;
  elevation_End_degree = 0;
  
  //Initialization of Gyros information 
  
  Gx=0;
  Gy=0;
  Gz=0;
  sens=0.00875;
  dt = 0.03;
  user_command = 0;
  LCD_count = 0;

  
  //****** initialization end here ******
  // Main function start 
  
  
  //Welcome Message diplay in the terminal 
  SCI1_OutString("--- Group-9 MTRX2700 Majot Project ---");
  SCI1_OutString("\r\n");

   //Infinity loop 
   while(1){
    
    PC_or_board = 0;
    selected_mode = 0;                 // Initialization of the both select mode

    Commend_configure();
    
    // If mode 2 select 
    if(user_command == 2){

      // Display help method
      Help_function();

    // If mode 1 select
    }else if(user_command == 1){


      // Prepare for PWM function 
      PWMDTY45 = 2250;
      PWMDTY67 = 2250;
      delay1(100);
    
      Gz=0;
      // Prepare for Gyro function
      delay1(100);
      Gx=0;
      Gy=0;
      
      // Selecting main functional mode 
      MainInput_moudle_Select();
    
        //Using PC Interface;
       //If chossing serial port as input
      if(PC_or_board == 1){
      
         
        // Enter terminal input format
        PC_demo_modeSelect();
        
        // if functional mode 1 selected start tracking 
        if(selected_mode == 1){
          PWM_init();
          PWM_SCI_Tracking();
          
        // if functional mode 2 selected start mapping&plotting  
        }else if(selected_mode == 2){
          PWM_init();
          PWM_mapping();
        
        }else{
        // if functional mode 3 selected start orientation demonstration 
          Orientation_demo_modle();
        }
        
      }else{
      
         // Using Keypad Interface;
        // if choosing keypad as input 
        keypad_mode_angle();
        
        // if functional mode 1 selected start tracking
        if(selected_mode == 1){
        
        // if functional mode 2 selected start mapping&plotting
          SCI1_OutString("\r\nStart Tracking...\r\n");
          PWM_init();
          PWM_SCI_Tracking();
        
          
        }else if(selected_mode == 2){
          
          SCI1_OutString("\r\nStart Mapping...\r\n");
          PWM_init();
          PWM_mapping();
          
        }else{
        
         // if functional mode 3 selected start orientation demonstration 
          Orientation_demo_modle();
        }
      }

    }else{

      // Exit the program
      SCI1_OutString("\r\n---- Thanks for using Group-9 MP Interface! ----\r\n");
      break;
    }
  } 
  
}


//   ***********************************************************  END Main   *****************************************************



// *********************************************************** Command Configure interface **************************************

void Commend_configure(void){
  do{
  SCI1_OutString("\r\nUser Command Configureation:\r\n(1) Funcationality\r\n(2) Help\r\n(3) Quit\r\n");
    SCI1_OutString("Moudle No.:");
    
    user_command = SCI1_InUDec();
    
    if(user_command != 1 && user_command != 2 && user_command != 3){
    
      SCI1_OutString("\r\nInvalid Moudle Command!, Command No. should be 1 ,2 or 3\r\n");
      SCI1_OutString("\r\n");
    }
    
  }while(user_command != 1 && user_command != 2 && user_command != 3);
  
  SCI1_OutString("\r\n");


}

// *********************************************************** Help function ***************************************************

void Help_function(void){

  SCI1_OutString("\r\n  ---- User Manual ----  \r\n");
  SCI1_OutString(" ");
  SCI1_OutString("** User Command **\r\n\r\n(1) Funcationality\r\n    - Mapping System Configureation\r\n");
  SCI1_OutString("      1.Using PC(Serial)\r\n      2.Using Dragon 12 Board (Keypad)\r\n");
  SCI1_OutString("\r\n(2) Help -- Show User Manual\r\n");
  SCI1_OutString("\r\n(3) Quit -- End the program\r\n");
  SCI1_OutString(" ");
  SCI1_OutString(" ------------------------------ ");
  SCI1_OutString(" \r\n");
  SCI1_OutString("Either Interface(PC or Board) has funcationality:\r\n");
  SCI1_OutString("\r\n  1. Tracking\r\n    - Select Azimuth Traget Degree (Available Range: 30 ~ 160)\r\n");
  SCI1_OutString("    - Select Elevation Traget Degree (Available Range: -60 ~ 60)\r\n");
  SCI1_OutString("\r\n  2. Scan & Plotting\r\n    - Select Azimuth Start & End Degree (Available Range: 30 ~ 160)\r\n");
  SCI1_OutString("    - Select Elevation Start & End Degree (Available Range: -60 ~ 60)\r\n");
  SCI1_OutString("    - Select Step Change (Avaliable input: 1, 2, 4 or 5 degree per step)\r\n");
  SCI1_OutString("    - Select Number of Sample per Oirnetation (Avaliable input: 1, 2 or 3 sample per Orien)\r\n");
  SCI1_OutString("    - Select Sample Frequency (Avaliable input: 10 ~ 50 Hz)\r\n");
  SCI1_OutString("\r\n  3. Orientation Demostration\r\n");
  SCI1_OutString("    - Please Operating on Board\r\n");
  SCI1_OutString(" ");
  SCI1_OutString(" ------------------------------ ");
  SCI1_OutString(" ");
  SCI1_OutString("\r\n** KeyPad Manual **\r\n");
  SCI1_OutString("  - Button 1 ~ 9 corresponding to Input Number 1 ~ 9\r\n");
  SCI1_OutString("  - Button * corresponding to '-'\r\n");

}
//*************************************************************FUNCTIONS USING BELOW**********************************************

void MainInput_moudle_Select(void){

  do{
  
    SCI1_OutString("\r\nMapping System Configuration:\r\n(1) PC\r\n(2) Dragon 12 Board (Keypad)\r\n");
    SCI1_OutString("Moudle No.:");
    
    PC_or_board = SCI1_InUDec();
    
    if(PC_or_board != 1 && PC_or_board != 2){
    
      SCI1_OutString("\r\nInvalid Moudle No.!, No. should be 1 or 2\r\n");
      SCI1_OutString("\r\n");
    }
    
  }while(PC_or_board != 1 && PC_or_board != 2);
  
  SCI1_OutString("\r\n");
  SCI1_OutString("Moudle Selected...\r\n");
  
}


//   ***************** PC Moudle Select  *****************

int PC_demo_modeSelect(void){

  int mode1 = 0;
  SCI1_OutString("\r\n--- PC Funcationality ---");
  SCI1_OutString("\r\n");

  do{

    SCI1_OutString("Moudle List:\r\n(1) Tracking\r\n(2) Scan & Plotting\r\n(3) Orentation Demostration");
    SCI1_OutString("\r\n");
    SCI1_OutString("Moudle No.:");

    mode1 = SCI1_InUDec();


    if(mode1 != 1 && mode1 != 2 && mode1 != 3){

      SCI1_OutString("\r\nInvalid Moudle No.!, No. should be 1,2 or 3\r\n");
    }


  }while(mode1 != 1 && mode1 != 2 && mode1 != 3);
  SCI1_OutString("\r\n");
  SCI1_OutString("Moudle Selected...\r\n\r\n");
  selected_mode = mode1;
}
//   ***************** Keypad Moudle Select  *****************

void keypad_mode_angle(void){

  selected_mode = LCD_welcome();
  
  PORTB = lookupDigits[tempnum];
  PTP = 0b1110;
  
  // Azimuth Start
  azimuth_Start_degree = Scan_Azimuth_start();
  SCI1_OutString("\r\nBoard Model Azimuth Start degree:\r\n");
  SCI1_OutUDec((unsigned short) azimuth_Start_degree);
  
   // Azimuth End
  azimuth_End_degree = Scan_Azimuth_end();
  SCI1_OutString("\r\nBoard Model Azimuth End degree:\r\n");
  SCI1_OutUDec((unsigned short) azimuth_End_degree);
  
  
  
  
  // Elevation Start
  elevation_Start_degree = Scan_Elevation_start();
  SCI1_OutString("\r\nBoard Model Elevation Start degree:\r\n");
  
  if(elevation_Start_degree < 0){
    SCI1_OutString("-");
    SCI1_OutUDec((unsigned short) abs(elevation_Start_degree));
  }else{
    SCI1_OutUDec((unsigned short) elevation_Start_degree);
  }
  
  // Elevation End
  elevation_End_degree = Scan_Elevation_end();
  SCI1_OutString("\r\nBoard Model Elevation End degree:\r\n");
  
  if(elevation_End_degree < 0){
    SCI1_OutString("-");
    SCI1_OutUDec((unsigned short) abs(elevation_End_degree));
  }else{
    SCI1_OutUDec((unsigned short) elevation_End_degree);
  }
  
  Scan_NumberOfSample();
  Scan_StepChange();
  Scan_SampleFrequency();
  
  resolution = 5;

}
 /*
void DIP_Switch_init(void){

  // Configure Port H (DIP switch Interrupt) to input
  DDRH = 0x00;

  // Enable the Deep Switch Interrupt
  PIEH = 0xFF;

  // Enable the capture the rasing edge as interrupt trigger
  PERH = 0xFF;

}
 */
//  ********************************************************  Orientation_demo_modle  ***********************************************

void Orientation_demo_modle(void){

  int display_Sensor_value = Gx;
  sensor_offset();

  DIP_display_No = 1;
  PWM_OFF();
  //DIP_Switch_init();

  DIP_Model_quit_flag = 1;
  
  DIP_change_Different_flag = 1;
  
  while(1){
    
      if(LCD_count==100){
          LCD_count = 0;
          DIP_change_Different_flag = 1;
          DIP_display_No +=1;
          
      }
      
      
      if(DIP_Model_quit_flag == 1){
      
      
        if(DIP_change_Different_flag){
        
          if(DIP_display_No > 6){
            DIP_display_No = 1;
          }
        
           switch(DIP_display_No){
           
             case 1:
              gyro_heading_lcd_sentense();
              
              break;

             case 2:
              gyro_elevation_lcd_sentense();
              break;

             case 3:
              accel_elevation_lcd_sentense();
              
              break;

             case 4:
              
              break;

             case 5:
              servo_heading_lcd_sentense();
              break;

             case 6:
              servo_elevation_lcd_sentense();
              break;
          }
        
          DIP_change_Different_flag = 0;
        }

        switch(DIP_display_No){
           
           case 1:
            data_process();
            display_Sensor_value = Gx;
            gyro_heading_lcd_data(display_Sensor_value);
            msDelay(10);
            LCD_count+=1;

            break;

          case 2:
            data_process();
            display_Sensor_value = Gy;
            gyro_elevation_lcd_data(display_Sensor_value);
            msDelay(10);
            LCD_count+=1;
            break;

          case 3:
            data_process();
            display_Sensor_value = theta;
            accel_elevation_lcd_data(display_Sensor_value);
            msDelay(10);
            LCD_count+=1;
            break;

          case 4:
            data_process();
            display_Sensor_value = polar;
            msDelay(10);
            LCD_count+=1;
            break;

          case 5:
            display_Sensor_value = (PWMDTY67 - 900)/15;
            servo_heading_lcd_data(display_Sensor_value);
            msDelay(10);
            LCD_count+=1;
            
            break;

          case 6:
            display_Sensor_value = (PWMDTY45-2250)/15;
            servo_elevation_lcd_data(display_Sensor_value);
            msDelay(10);
            LCD_count+=1;

            break;
        }
      }
  }
}



//  ******************************************************************  PWM   *******************************************************

void PWM_init(void){

  // All channel select the clock A or B;
  PWMCLK = 0x00;

  // Select the polarity of the channel 6 and 4 to high, channel 7 and 5 to low.
  PWMPOL = 0xFF;

  // Select the clock A and B with pre scale 16
  PWMPRCLK = 0x44;

  // Concatenate the channel 4 and 5, 6 and 7.
  PWMCTL = 0xC0;

  // Set all PWM channel output to be left aligened
  PWMCAE = 0x00;

  // Set the period of the channel 4,5 and 6,7 to 50Hz (24M/16 --> 1.5M/30000 --> 50Hz)
  PWMPER45 = 30000;
  PWMPER67 = 30000;

  // Set the init duty cycle of the channel 4,5 and 6,7 to 7.5%
  PWMDTY45 = 2250;
  PWMDTY67 = 2250;

  //PWM Enable the channle 4,5 6 and 7
  PWME = 0xF0;

  // Delay 50 ms
  delay1(50);
}

void PWM_OFF(void){

  //PWM Enable the channle 4,5 6 and 7
  PWME = 0x00;
  delay1(50);
}
//*****************************   Tracking *************************************

void PWM_SCI_Tracking(void){

  //unsigned short resolution;

  int ErrorFlag = 0;

  signed int iterator1 = 0;
  signed int iterator2 = 0;

  // Interval of the Azimuth Servo degree (30~160) --> (30 ~ 160) (1.067ms ~ 1.9333ms) (DTY:1650 ~ 2950 (0 error))
  
  if(PC_or_board == 1){
  
    SCI1_OutString("\r\n--- PWM Tracking & Mapping Orentation Selection ---");
    SCI1_OutString("\r\n");

    SCI1_OutString("Azimuth start from (30-160):");

    do{

      azimuth_Start_degree = SCI1_InSDec();

      if(azimuth_Start_degree > 160 || azimuth_Start_degree < 30){
        SCI1_OutString("\r\n");
        SCI1_OutString("Invalid Degree (degree should in range: 30~160)");
        SCI1_OutString("\r\n");
        ErrorFlag = 1;
       }
       else{
        ErrorFlag = 0;
        azimuth_Start_DCycle = (10 * (azimuth_Start_degree)) +1350 - (90 - azimuth_Start_degree)* 5;
        //azimuth_Start_DCycle = (10 * (azimuth_Start_degree)) +1350;
       }

    }while(ErrorFlag);
    
    SCI1_OutString("\r\n");

    if(selected_mode == 2){
      SCI1_OutString("End from (");

      SCI1_OutUDec((unsigned short) azimuth_Start_degree + 1);
      
      SCI1_OutString("~160):");

      do{

        azimuth_End_degree = SCI1_InSDec();

        if(azimuth_End_degree > 160 || azimuth_End_degree < azimuth_Start_degree){
          SCI1_OutString("\r\n");
          SCI1_OutString("Invalid Degree (degree should in range: ");
          SCI1_OutUDec((unsigned short) azimuth_Start_degree + 1);
          SCI1_OutString("~160)");
          SCI1_OutString("\r\n");
          ErrorFlag = 1;
         }
         else{
          ErrorFlag = 0;
          azimuth_End_DCycle = (10 * (azimuth_End_degree)) +1350 - (90 - azimuth_End_degree)* 5;
          //azimuth_End_DCycle = (10 * (azimuth_End_degree)) +1350;
         }

      }while(ErrorFlag);
      SCI1_OutString("\r\n");
    }
    

     // Interval of the Azimuth Servo degree (-60 ~ 60) (1.1ms ~ 1.9ms) (DTY:1650 ~ 2850 (0 error))
    SCI1_OutString("Elevation start from (-60~60):");
    
    do{

      elevation_Start_degree = SCI1_InSDec();

      if(elevation_Start_degree > 60 || elevation_Start_degree < -60){
        SCI1_OutString("\r\n");
        SCI1_OutString("Invalid Degree (degree should in range: -60~60)");
        SCI1_OutString("\r\n");
        ErrorFlag = 1;
       }
       else{
        ErrorFlag = 0;
        elevation_Start_DCycle = (10 * (elevation_Start_degree)) +2250 - (0 - (elevation_Start_degree)) * 5;
        //elevation_Start_DCycle = (10 * (elevation_Start_degree)) +2250;
       }

    }while(ErrorFlag);
    SCI1_OutString("\r\n");


    if(selected_mode == 2){
      
      SCI1_OutString("End from (");
      
      if(elevation_Start_degree < 0){
        SCI1_OutString("-");
        SCI1_OutUDec((unsigned short) abs(elevation_Start_degree + 1));
      }else{
        SCI1_OutUDec((unsigned short) elevation_Start_degree + 1);
      }
      SCI1_OutString("~60):");
      
      
      do{

        elevation_End_degree = SCI1_InSDec();

        if(elevation_End_degree > 60 || elevation_End_degree < elevation_Start_degree){
          SCI1_OutString("\r\n");
          SCI1_OutString("Invalid Degree (degree should in range: ");
          SCI1_OutUDec((unsigned short) elevation_Start_degree + 1);
          SCI1_OutString("~60):");
          SCI1_OutString("\r\n");
          ErrorFlag = 1;
         }
         else{
          ErrorFlag = 0;
          elevation_End_DCycle = (10 * (elevation_End_degree)) +2250 - (0 - (elevation_End_degree)) * 5;
          //elevation_End_DCycle = (10 * (elevation_End_degree)) +2250;
         }

      }while(ErrorFlag);
      SCI1_OutString("\r\n");
    }
    

    // Resolution 1 ~ 5 (duty: 10:50, step:10)
    SCI1_OutString("Step change (1-5) -> (0.1 ~ 0.5 degree per orien):");
    do{

      resolution = SCI1_InSDec();

      if(resolution > 5 || resolution < 1){
        SCI1_OutString("\r\n");
        SCI1_OutString("Invalid Step change (should in range: 1~5)");
        SCI1_OutString("\r\n");
        ErrorFlag = 1;
       }
       else{
        ErrorFlag = 0;
       }

    }while(ErrorFlag);
    SCI1_OutString("\r\n");
    
    

  }else {
  
    SCI1_OutString("Duty Cycle calculating...");
    SCI1_OutString("\r\n");
    
    azimuth_Start_DCycle = (10 * (azimuth_Start_degree)) +1350 - (90 - azimuth_Start_degree)* 5;
    
    azimuth_End_DCycle = (10 * (azimuth_End_degree)) +1350 - (90 - azimuth_End_degree)* 5;
    
    elevation_Start_DCycle = (10 * (elevation_Start_degree)) +2250 - (0 - (elevation_Start_degree)) * 5;
    
    elevation_End_DCycle = (10 * (elevation_End_degree)) +2250 - (0 - (elevation_End_degree)) * 5;
    
    
    SCI1_OutString("azimuth_Start_DCycle: ");
    SCI1_OutUDec((unsigned short) azimuth_Start_DCycle);
    SCI1_OutString("\r\n");
    
    SCI1_OutString("azimuth_End_DCycle: ");
    SCI1_OutUDec((unsigned short) azimuth_End_DCycle);
    SCI1_OutString("\r\n");
    
    
    SCI1_OutString("elevation_Start_DCycle: ");
    SCI1_OutUDec((unsigned short) elevation_Start_DCycle);
    SCI1_OutString("\r\n");
    
    SCI1_OutString("elevation_End_DCycle: ");
    SCI1_OutUDec((unsigned short) elevation_End_DCycle);
    SCI1_OutString("\r\n");
  }
  

// Start Moving the PTU to target Orientation...

  iterator1 = PWMDTY67;
  iterator2 = PWMDTY45;
  
  
  if(iterator1 < azimuth_Start_DCycle){

    while(iterator1 < azimuth_Start_DCycle){
    
      // Check degree changing
      
      /*SCI1_OutUDec((unsigned short) (90 - Gx));
      SCI1_OutString(",");
      
      if(Gy < 0){
        SCI1_OutString("-");
        SCI1_OutUDec(abs(Gy));
        SCI1_OutString("\r\n");
      }else{
        SCI1_OutUDec((unsigned short) Gy);
        SCI1_OutString("\r\n");
      }*/
      
      SCI1_OutUDec((unsigned short) PWMDTY67);
      SCI1_OutString(",");
      SCI1_OutUDec((unsigned short) azimuth_Start_DCycle);
      SCI1_OutString("\r\n");
      PWMDTY67 += 5;
      iterator1 += 5;
      delay1(100);
    }
  }else{

    while(iterator1 > azimuth_Start_DCycle){
    
      // Check degree changing
      
     /* SCI1_OutUDec((unsigned short) (90 - Gx));
      SCI1_OutString(",");
      
      if(Gy < 0){
        SCI1_OutString("-");
        SCI1_OutUDec(abs(Gy));
        SCI1_OutString("\r\n");
      }else{
        SCI1_OutUDec((unsigned short) Gy);
        SCI1_OutString("\r\n");
      }
      */
      SCI1_OutUDec((unsigned short) PWMDTY67);
      SCI1_OutString(",");
      SCI1_OutUDec((unsigned short) azimuth_Start_DCycle);
      SCI1_OutString("\r\n");
      
      PWMDTY67 -= 5;
      iterator1 -= 5;
      
      delay1(100);
    }
  }

  if(iterator2 < elevation_Start_DCycle){

    while(iterator2 < elevation_Start_DCycle){
    
      // Check degree changing
      
      /*SCI1_OutUDec((unsigned short) (90 - Gx));
      SCI1_OutString(",");
      
      if(Gy < 0){
        SCI1_OutString("-");
        SCI1_OutUDec(abs(Gy));
        SCI1_OutString("\r\n");
      }else{
        SCI1_OutUDec((unsigned short) Gy);
        SCI1_OutString("\r\n");
      }
      */
      SCI1_OutUDec((unsigned short) PWMDTY45);
      SCI1_OutString(",");
      SCI1_OutUDec((unsigned short) elevation_Start_DCycle);
      SCI1_OutString("\r\n");
      PWMDTY45 += 5;
      iterator2 += 5;
      delay1(100);
    }
  }else{

    while(iterator2 > elevation_Start_DCycle){
    
      // Check degree changing
      
      /*SCI1_OutUDec((unsigned short) (90 - Gx));
      SCI1_OutString(",");
      
      if(Gy < 0){
        SCI1_OutString("-");
        SCI1_OutUDec(abs(Gy));
        SCI1_OutString("\r\n");
      }else{
        SCI1_OutUDec((unsigned short) Gy);
        SCI1_OutString("\r\n");
      } */
      
      SCI1_OutUDec((unsigned short) PWMDTY45);
      SCI1_OutString(",");
      SCI1_OutUDec((unsigned short) elevation_Start_DCycle);
      SCI1_OutString("\r\n");
      PWMDTY45 -= 5;
      iterator2 -= 5;
      delay1(100);
    }
  }
  
}
 
//*****************************   Mapping Mode   *************************************

void PWM_mapping(void){

  double Degree_to_Radian = (PI / 180);
  int aDistance = 0;
  int eDistance = 0;
  unsigned short true_dis = 0;
  int ele_aim_cycle = 0;
  int iterator1 = 0;
  int iterator2 = 0;
  int current_elev_degree = 0;
  int current_az_degree = 0;
  
  // Convert distance --> unsigned short
  unsigned short distance_copy = 0;


  PWM_SCI_Tracking();
  
  iterator1 = azimuth_Start_DCycle;
  iterator2 = elevation_Start_DCycle;

  
  ele_aim_cycle = elevation_End_DCycle;
  
   // offline mode: Azimuth degree 40 to 140, Elevation degree -40 to 40, step: 0.2~ 1 degree change PER STEP
   
   while(iterator1 < azimuth_End_DCycle){
   
    

    if(ele_aim_cycle == elevation_End_DCycle){
       
       
       while(iterator2 <= ele_aim_cycle){

        distance_copy = distance;
        current_az_degree = (iterator1 - 900)/15;
        current_elev_degree =(iterator2 - 2250)/15;
        

        aDistance = cos(current_az_degree * Degree_to_Radian) * (distance_copy) * cos(current_elev_degree * Degree_to_Radian);

        eDistance = sin(current_elev_degree * Degree_to_Radian) * (distance_copy);

        if(aDistance >= 0){
          SCI1_OutUDec((unsigned short) aDistance);
        }else{
          SCI1_OutString("-");
          SCI1_OutUDec((unsigned short) (- aDistance));
        }

        true_dis = (distance_copy) * cos(current_elev_degree * Degree_to_Radian) * sin(current_az_degree * Degree_to_Radian);

        SCI1_OutString(",");

        // Acelerometer z axis

        if(eDistance >= 0){
          SCI1_OutUDec((unsigned short) eDistance);
        }else{
          SCI1_OutString("-");
          SCI1_OutUDec((unsigned short) (- eDistance));
        }

        SCI1_OutString(",");

        //laser Distance

        SCI1_OutUDec((unsigned short) true_dis);
        SCI1_OutString("\r\n");

        PWMDTY45 += (2 * resolution) + 4;
        
        delay1(50);
         
        iterator2 += (2 * resolution) + 4;

      }

    }else{

      while(iterator2 >= ele_aim_cycle){

        distance_copy = distance;

        current_az_degree = (iterator1 - 900)/15;
        current_elev_degree =(iterator2 - 2250)/15;



        aDistance = cos(current_az_degree * Degree_to_Radian) * (distance_copy) * cos(current_elev_degree * Degree_to_Radian);

        eDistance = sin(current_elev_degree * Degree_to_Radian) * (distance_copy);
        
        if(aDistance >= 0){
          SCI1_OutUDec((unsigned short) aDistance);
        }else{
          SCI1_OutString("-");
          SCI1_OutUDec((unsigned short) (- aDistance));
        }

        true_dis = (distance_copy) * cos(current_elev_degree * Degree_to_Radian) * sin(current_az_degree * Degree_to_Radian);

        SCI1_OutString(",");

        // Acelerometer z axis

        if(eDistance >= 0){
          SCI1_OutUDec((unsigned short) eDistance);
        }else{
          SCI1_OutString("-");
          SCI1_OutUDec((unsigned short) (- eDistance));
        }

        SCI1_OutString(",");

        //laser Distance

        SCI1_OutUDec((unsigned short) true_dis);
        SCI1_OutString("\r\n");

        PWMDTY45 -= (2 * resolution) + 4;
        delay1(50);
         
        iterator2 -= (2 * resolution) + 4;
      }
       
    }
    
      if(ele_aim_cycle == elevation_Start_DCycle){
        ele_aim_cycle = elevation_End_DCycle;
      }else{
        ele_aim_cycle = elevation_Start_DCycle;
      }
      
      PWMDTY67 += 2 * resolution;
      //PWMDTY67 += resolution;
      delay1(50);
      //iterator1 += resolution;
      
      //current_az_degree += (resolution/5);
      iterator1 += 2 * resolution;
 
    }

    
    PWMDTY67 = 2250;
    PWMDTY45 = 2250;
    Gx=0;
    Gy=0;
    Gz=0;
}
//  *****************************************************  PWM END  ****************************************************

//  *****************************************************  TIMER INIT  **************************************************

void Init_TIMERS (void) {

_asm SEI;

TSCR1=0x80;     //enable the timer
TSCR2 = 0x80; // enable timer overflow interrupt ,prescaler 1

TIOS= TIOS|0xFD;        //only pt1 is input capture

TIE=TIE | 0x42;    //Timer interrupt enbable, PT1, PT6,PT5
TCTL4 = 0X04;
 _asm CLI;

}



// //*******************************************3 SENSERS INIT AND Catch**********************************************

void magnet_init(void){

  int  res1;
  res1=iicstart(magnet_wr);
  res1=iictransmit(HM5883_MODE_REG );  //
  res1=iictransmit(0x00 );
  iicstop();

}


void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw){
 double Mz,My;
 uint8_t i = 0;
 uint8_t buff[6];
 int res1;

 res1=iicstart(magnet_wr);
 res1=iictransmit(HM5883_DATAX0 );
 res1= iicrestart(magnet_rd);
 iicswrcv();

 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }

 buff[i]= iicreceivem1();
 buff[i+1]= iicreceivelast();

	*mxraw = ((buff[0] << 8) | buff[1]);
	*myraw = ((buff[2] << 8) | buff[3]);
	*mzraw = ((buff[4] << 8) | buff[5]);
	
	Mz = (mzraw[0]-49.5)*0.686654;
	My = (myraw[0]-1.5)*0.744235;
	polar = -atan2(My,Mz)*180/PI;
	position2 = polar-position1;
	/*
	if(polar<-180){
	  polar+=360;
	}
	*/
  
}



//Accelometer

void accel_init (void){

 int  res1;

 res1=iicstart(accel_wr);
 res1=iictransmit(ADXL345_POWER_CTL );
 res1=iictransmit(0x08 );

 res1=iictransmit(ADXL345_DATA_FORMAT );
 res1=iictransmit(0x08 );

 iicstop();
}


void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw){

 uint8_t i = 0;
 uint8_t buff[6];
 int res1;

 res1=iicstart(accel_wr);
 res1=iictransmit(ADXL345_DATAX0 );
 res1= iicrestart(accel_rd);
 iicswrcv();

 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }

 buff[i]= iicreceivem1();
 buff[i+1]= iicreceivelast();

	*axraw = ((buff[1] << 8) | buff[0]);
	*ayraw = ((buff[3] << 8) | buff[2]);
	*azraw = ((buff[5] << 8) | buff[4]);

	Ax = axraw[0]*mGperLSB;
  Ay = ayraw[0]*mGperLSB;
	Az = azraw[0]*mGperLSB;

	theta = atan(Az/sqrt(Ay*Ay+Ax*Ax));
	theta =theta*180/PI;


}




 //  Gyro Initialisation

 void gyro_init (void) {

 int  res1;

 res1=iicstart(gyro_wr);
 res1=iictransmit(L3G4200D_CTRL_REG1 );  // ; 100hz, 12.5Hz, Power up
 res1=iictransmit(0x0f );
 iicstop();
 }


// Function to get a set of gyro data
// Eduardo Nebot,30 July 2015

void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw) {
 	uint8_t i = 0;
	uint8_t buff[6];
	int res1;

   res1=iicstart(gyro_wr);
   res1=iictransmit(L3G4200D_OUT_XYZ_CONT );
   res1= iicrestart(gyro_rd);

 iicswrcv();

 for(i=0; i<4  ;i++) {
  buff[i]=iicreceive();
 }

buff[i]= iicreceivem1();
buff[i+1]= iicreceivelast();

	*gxraw = ((buff[1] << 8) | buff[0]);
	*gyraw = ((buff[3] << 8) | buff[2]);
	*gzraw = ((buff[5] << 8) | buff[4]);
	
  Gy = 0.98*(Gy+gyraw[0]*dt*sens)+0.02*(theta);
  Gx = 0.98*(Gx+gxraw[0]*dt*sens)+0.02*(position2);	
}



void setAlarm1(uint16_t msDelay1)
{
    alarmTime1 = currentTime1 + msDelay1;
    alarmSet1 = 1;
    alarmSignaled1 = 0;
}


void delay1(uint16_t msec)
{
    TC6 = TCNT + 24000; // Set initial time
    setAlarm1(msec);
    while(!alarmSignaled1) {};
}



/************************************ 7 segment display ***************************************/

void segments(void){

  // config 7 Segment Display
  // variable definition

  hunds = distance/10%1000/100;
  PTP = windows[1];
  PORTB = lookupDigits[hunds];

  msDelay(3);

  ttens = distance/10%100/10;
  PTP = windows[2];
  PORTB = lookupDigits[ttens];
  msDelay(3);

  ddigits = distance/10%10;
  PTP = windows[3];
  PORTB = lookupDigits[ddigits];

  msDelay(3);

}


/************************************ data_get  ***************************************/
void sensor_offset(void){
   hm5883_getrawdata(&mxraw, &myraw, &mzraw);
   position1 = polar;   
}
void data_process(void){
   int i;
   hm5883_getrawdata(&mxraw, &myraw, &mzraw);
   adxl345_getrawdata(&axraw,&ayraw,&azraw);
   
   for(i=0; i < 5; i++)  {
      l3g4200d_getrawdata(&gxraw, &gyraw, &gzraw);
   }

}


/**************************************************************** HERE IS Interrupt EMN **************************************************************/

// interrupt(((0x10000-Vtimch7)/2)-1) void TC7_ISR(void){
// the line above is to make it portable between differen
// Freescale processors
// The symbols for each interrupt ( in this case Vtimch7 )'
// are defined in the provided variable definition file
// I am usign a much simpler definition ( vector number)
// that is easier to understand

interrupt 14 void TC6_ISR(void) {

  TC6 =TCNT + 24000;   // interrupt every msec
  TFLG1=TFLG1 | TFLG1_C6F_MASK;
  currentTime1++;
    if (alarmSet1 && currentTime1 == alarmTime1)
    {
        alarmSignaled1 = 1;
        alarmSet1 = 0;
    }
}


// This interrupt calculate pulse width and store the distance in
interrupt 9 void TC1_ISR(void){
  //Clear the interrupt flag of PT1
  TFLG1 = 0x02;
  if(is_rising == 0){

     overflow = 0;
     //Get the current count as rasing edge
     edge_rising = TC1;
     //Set raise flag to 1,raising edge has been stored in distance
     is_rising = 1;
     //Prepare to capture the falling edge
     TCTL4 = 0x08;

  }else if(is_rising == 1){

     //Get the current count as falling edge
     edge_falling = TC1;
     //Set raise flag back to 0 for next calculation
     is_rising = 0;
     //Prepare to capture the rasing edge again
     TCTL4 = 0x04;
     //Calculate the cycles between edges
     if(edge_falling > edge_rising){
       distance = ((overflow*65536 + (edge_falling - edge_rising))/24)-100;
     }else if(edge_falling < edge_rising){
       distance = ((overflow*65536 - (edge_rising - edge_falling))/24)-100;
     }
     
     if(count>=5){      // Display the number each time it get values 
        count = 0;
        segments(); 
     }
     count+=1;
     
  }
  
}

//This interrupt calculate the times timer overflow
interrupt 16 void Overflow_ISR(void){
   TFLG2 = TFLG2 | TFLG2_TOF_MASK;
   overflow++;
}

// ******************* DIP SWITCH **********************
/*
interrupt 25 void DIP_Switch(void){

  PIFH =  0xFF;
  PTH_value = PTH;

  switch(PTH_value){

    case 0x01:
      //DIP_Model_quit_flag = 0;
      break;

    case 0xC0:
      
      if(DIP_display_No != 2){
        DIP_change_Different_flag = 1;
      }
      
      DIP_display_No = 2;
      break;

    case 0xE0:
      
      if(DIP_display_No != 3){
        DIP_change_Different_flag = 1;
      }
      
      DIP_display_No = 3;
      break;

    case 0xF0:
    
      if(DIP_display_No != 4){
        DIP_change_Different_flag = 1;
      }
      
      DIP_display_No = 4;
      break;

    case 0xF8:
      
      if(DIP_display_No != 5){
        DIP_change_Different_flag = 1;
      }
      
      DIP_display_No = 5;
      break;

    case 0xFC:
      
      if(DIP_display_No != 6){
        DIP_change_Different_flag = 1;
      }
      
      DIP_display_No = 6;
      break;

    default:
      
      if(DIP_display_No != 1){
        DIP_change_Different_flag = 1;
      }
      
      DIP_display_No = 1;
      break;
  }
}
             */
//TC5 getting values from gyro
/*
interrupt 13 void TC5_ISR(void){

    TC5 = TC5 +65535;
    TFLG1=0x20;
    l3g4200d_getrawdata(&gxraw, &gyraw, &gzraw);
       gxraw[0]-=80;
       Gx=Gx+gxraw[0]*0.00273*sens;
       gyraw[0]-=11;
       Gy=Gy+gyraw[0]*0.00273*sens;
       gzraw[0]+=34;
       Gz=Gz+gzraw[0]*0.00273*sens;
}
   */