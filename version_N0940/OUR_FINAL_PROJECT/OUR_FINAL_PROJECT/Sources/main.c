// Demonstration functions for IIC to read inertial sensor values
//
// The program will send read values to the serial port.
// you need to connect the serial port to a terminal to verify operation
// port speed 9600 bauds
//
// Eduardo Nebot  29 January 2016
// Sensors implemented:
// Gryro L2g2400D  (3 axis )
// Accelerometer  ADXL345
// Magnetometer HM5883
// Laser Lidarlight:  the driver is working with version 1 but not with version 2
// Version 2 work in progress: 
// Laser Interface will be done by measuring the pulse PWM pulse width
// Last version: 29/1/16
// 
//  This version installed interrupts with the simple model
//  The user should create a project selecting small memory model
//  and minimal startup code. This form will not intiliase any variables !
//  Your program will need to intialize the variables !
//
// Resources used: This program is using Timer 6 for the sampling time
// 
// the iic drivers are using Timer 7. ( You cannot use this timer in your program)
// Do not change the prescaler. If you do you need to change some code in iic.c
//
//    

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

char sev_seg[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x67};   // Array of hex values representing 0-9 on the 7 seg display                    //
char windows[] = {0x0E,0x0D,0x0B,0x07};                                 // Corresponds to which display the value is required to be
             
             



void segments(void);
        

void adxl345_getrawdata(int *axraw, int *ayraw, int *azraw);
void accel_init(void);

void hm5883_getrawdata(int *mxraw, int *myraw, int *mzraw);
void magnet_init(void);

void l3g4200d_getrawdata(int *gxraw, int *gyraw, int *gzraw);
void gyro_init(void);



unsigned char LidarWriteAddr = 0xc4;
uint8_t LidarReadAddr = 0xc5;
uint8_t Lidar2ByteRead = 0x8f;

uint16_t Dist;



// !!!!!!!! Charles Added PWM Function
#define PI  3.14159265
int k;
int mode;
int demo_modeSelect(void);
void PWM_init(void);
void PWM_SCI_offline_mapping(void);
void PWM_SCI_realTime_mapping(void);
void keypad_mode_angle(void);


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


// Haijun Zeng 7 segment
int thous, hunds, ttens, ddigits;


int tempnum; 

int lookupDigits[14]; 


int azimuth_Start_degree;
int azimuth_End_degree;
   
int elevation_Start_degree;
int elevation_End_degree;



// ***********************************************************MAIN LOOP HERE*********************************************************************



void main(void) {
    /* put your own code here */
    
   
   
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
   // !!!!!!!!Hongzhan Yu modified in 24/05/2018
   overflow=0;
   is_rising=0;
   edge_rising=0;
   edge_falling=0;
   distance=0;
   rotation=0;
   
   
   // initialization here ******
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
  
   EnableInterrupts;
   
   
   
//infinite loop   
   
   
   
   

   //keypad_mode_angle(); 
     
     GOX=0;
     GOY=0;
     GOZ=0;
     Gx=0;
     Gy=0;
     Gz=0;
     sens=0.00875;
     
  PWM_SCI_offline_mapping();
  //keypad_mode_angle();
   
   while(1) {
    
       delay1(2); 
       // HM5883_magnetometer
       //l3g4200d_getrawdata(&gxraw, &gyraw, &gzraw);
       //hm5883_getrawdata(&mxraw, &myraw, &mzraw);
       //adxl345_getrawdata(&axraw,&ayraw,&azraw);
       
       
       
       SCI1_OutString(" Laser:");
       SCI1_OutUDec(distance);
       SCI1_OutString(" Gx:"); 
       
       SCI1_OutUDec((unsigned short) Gx) ;
       
       SCI1_OutString(" Gy:"); 
       SCI1_OutUDec((unsigned short) Gy) ;
       
       SCI1_OutString(" Gz:"); 
       SCI1_OutUDec((unsigned short) Gz) ;
      
       
       SCI1_OutString("\r\n");  
   }   
}

  
//   ***********************************************************  END Main   *****************************************************

//*************************************************************FUNCTIONS USING BELOW********************************************** 

//   ***************** Moudle Select  *****************

int demo_modeSelect(void){
  
  signed int select_mode;
  
  SCI1_OutString("--- Group-9 MTRX2700 Majot Project ---");
  SCI1_OutString("\r\n");
  
  do{
  
    
    
    SCI1_OutString("Moudle List:\r\n(1) Tracking\r\n(2) Scan & Plotting\r\n(3) Orentation Demostration");
    SCI1_OutString("\r\n");
    SCI1_OutString("Moudle No.:");
    
    select_mode = SCI1_InUDec();
    
    
    if(select_mode != 1 && select_mode != 2 && select_mode != 3){
      
      SCI1_OutString("\r\nInvalid Moudle No.!\r\n");
    }
    
    
  }while(select_mode != 1 && select_mode != 2 && select_mode != 3);
  
  return select_mode;
}
//   ***************** Keypad Moudle Select  *****************

void keypad_mode_angle(void){
  tempnum = LCD_welcome();
  PORTB = lookupDigits[tempnum]; 
  PTP = 0b1110;
  
  azimuth_Start_degree = Scan_Azimuth_start();
  azimuth_End_degree = Scan_Azimuth_end();
  elevation_Start_degree = Scan_Elevation_start();
  elevation_End_degree = Scan_Elevation_end();

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

//*****************************   Offline Mode   *************************************

void PWM_SCI_offline_mapping(void){


   unsigned int azimuth_Start_degree = (10 * (30)) +1365 - (90 - 30)* 7;
   unsigned int azimuth_End_degree = (10 * (160)) +1365 - (90 - 160)* 7;
   
   // PI/180 factor
   double Degree_to_Radian = (PI / 180);
  
  
  // Azimuth(x), Elevation(y), True_dis(z) axis 
   int aDistance = 0;
   int eDistance = 0;
   unsigned short true_dis = 0;
   
   
   // Convert distance --> unsigned short
   unsigned short distance_copy = 0;
   
   
   // Default degree of the mapping
   //int Az_degree = 30;
   //int Ev_degree = -37;
   
   
   // elevation mapping START/END duty cycle
   signed int elevation_Start_degree = (10 * (-25 - 20)) +2250;
   signed int elevation_End_degree = (10 * (25)) +2250;
   
   
   // Default changing step value
   unsigned short resolution = 5;
   
   
     
   int iterator1 = azimuth_Start_degree;
   int iterator2 = elevation_Start_degree;
   
   
   // Default offline mode: Azimuth degree 40 to 140, Elevation degree -40 to 40, step: 2 degree change
   
   //SCI1_OutString("Offline Mode, Getting point data....\r\n");
   
   PWMDTY67 = azimuth_Start_degree;
   PWMDTY45 = elevation_Start_degree;
   
   //delay1(100);
   
   
   while(iterator1 <= azimuth_End_degree){
   
      while(iterator2 <= elevation_End_degree){
      
       // SCI1_OutString("Getting point...");
        distance_copy = distance;
        
        // Assume Gyro is 40 degrees stable
        
        aDistance = sin(Gx * Degree_to_Radian) * (distance_copy) * cos(Gy * Degree_to_Radian);
        eDistance = sin(Gy * Degree_to_Radian) * (distance_copy);
     
           
          //aDistance = (90 - Gx);
          //eDistance = Gy;
          
       /* if(Ev_degree < 0 ){
          true_dis = (distance_copy - 90) * sin(Az_degree * Degree_to_Radian) * sin( (-Ev_degree) * Degree_to_Radian);
        }else{
          true_dis = (distance_copy - 90) * sin(Az_degree * Degree_to_Radian) * sin(Ev_degree * Degree_to_Radian);
        }
         */
         
        if(aDistance >= 0){
          SCI1_OutUDec((unsigned short) aDistance);
        }else{
          SCI1_OutString("-");
          SCI1_OutUDec((unsigned short) (- aDistance));
        } 
         
        true_dis = (distance_copy) * cos(Gy * Degree_to_Radian) * cos(Gx * Degree_to_Radian);
        
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
        
        
        delay1(20);
         
        //PWMDTY45 += (2 * resolution) + 4;
        PWMDTY45 += resolution + 4;
        //iterator2 += (2 * resolution) + 4;
        iterator2 += resolution + 4;
         
      }
      
      iterator2 = elevation_Start_degree; 
      
      delay1(20);
       
      //PWMDTY67 += (2 * resolution + 5);
      PWMDTY67 += resolution; 
      iterator1 += resolution;
      //iterator1 += (2 * resolution + 5);
      PWMDTY45 = elevation_Start_degree; 
   }
   
      
   //fclose(fp);
   
   PWMDTY67 = 2250;
   PWMDTY45 = 2250;
}

//*****************************   Real Time *************************************

void PWM_SCI_realTime_mapping(void){


   signed int azimuth_Start_degree;
   signed int azimuth_End_degree;
   
   signed int elevation_Start_degree;
   signed int elevation_End_degree;
   
   unsigned short resolution;
   
   signed int iterator1 = 0;
   signed int iterator2 = 0;
   
   // Interval of the Azimuth Servo degree (30~160) --> (30 ~ 160) (1.067ms ~ 1.9333ms) (DTY:1650 ~ 2950 (0 error)) 
   SCI1_OutString("--- PWM real-time mapping sub-system ---");
   SCI1_OutString("\r\n");
   
   SCI1_OutString("Azimuth start from (30-160):");
   azimuth_Start_degree = (10 * SCI1_InSDec()) +1300;
   SCI1_OutString("\r\n");
   
   SCI1_OutString("End from (30-160):");
   azimuth_End_degree = (10 * SCI1_InSDec()) +1300;
   SCI1_OutString("\r\n");
   
   // Interval of the Azimuth Servo degree (-60 ~ 60) (1.1ms ~ 1.9ms) (DTY:1650 ~ 2850 (0 error)) 
   SCI1_OutString("Azimuth start from (-60~60):");
   elevation_Start_degree = (10 * SCI1_InSDec()) +2250;
   SCI1_OutString("\r\n");
   
   SCI1_OutString("End from (-60~60):");
   elevation_End_degree = (10 * SCI1_InSDec()) +2250;
   SCI1_OutString("\r\n");
   
   
   
   // Resolution 1 ~ 5 (duty: 10:50, step:10)
   SCI1_OutString("Resolution Select (1-5):");
   resolution = (SCI1_InUDec() *10);
   SCI1_OutString("\r\n");
   
   
   
   PWMDTY67 = azimuth_Start_degree;
   PWMDTY45 = elevation_Start_degree;
   
   
   
   iterator1 = azimuth_Start_degree;
   iterator2 = elevation_Start_degree;
   
   while(iterator1 <= azimuth_End_degree){
   
      while(iterator2 <= elevation_End_degree){
      
        delay1(100); 
        PWMDTY45 += resolution;
        iterator2 += resolution; 
      }
      
      iterator2 = elevation_Start_degree; 
      
      delay1(100); 
      PWMDTY67 += resolution;
      iterator1 += (2 * resolution);
      PWMDTY45 = elevation_Start_degree; 
   }
   
      

   PWMDTY67 = 2250;
   PWMDTY45 = 2250;
}


//  *****************************************************  PWM END  **************************************************
//  *****************************************************  TIMER INIT  **************************************************

// !!!!!!!!Hongzhan Yu modified in 24/05/2018
void Init_TIMERS (void) {
  
_asm SEI;

TSCR1=0x80;     //enable the timer
TSCR2 = 0x80; // enable timer overflow interrupt ,prescaler 1

TIOS= TIOS|0x6D;        //only pt1 is input capture

TIE=TIE | 0x62;    //Timer interrupt enbable, PT1, PT6,PT5
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

 
                                                                 
 
  thous = distance/1000;                                            
  PTP = windows[0];
  PORTB = sev_seg[thous];   
 
  delay1(2);
  
  hunds = distance%1000/100;
  PTP = windows[1];
  PORTB = sev_seg[hunds];
  
  delay1(2);
  
  ttens = distance%100/10;
  PTP = windows[2];
  PORTB = sev_seg[ttens];
  delay1(2);
                                                    
  ddigits = distance%10;
  PTP = windows[3];
  PORTB = sev_seg[ddigits];    
                                                      
  delay1(2);                                                              
                                                                                                         
       
 
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

// !!!!!!!!Hongzhan Yu modified in 24/05/2018
// This interrupt calculate pulse width and store the distance in 
interrupt 9 void TC1_ISR(void){
  unsigned long dis;
  //Clear the interrupt flag of PT1
  TFLG1 = 0x02;
  if(is_rising == 0){
     
     overflow = 0;
     //Get the current count as rasing edge
     edge_rising = TC1;
     //Set raise flag to 1, implies raising edge has been stored in distance   
     is_rising = 1;
     //Prepare to capture the falling edge 
     TCTL4 = 0x08;
        
  }else if(is_rising == 1){
  
     //Get the current count as falling edge
     edge_falling = TC1;
     //Set raise flag back to 0 for next round of calculation
     is_rising = 0;
     //Prepare to capture the rasing edge again  
     TCTL4 = 0x04;
     //Calculate the difference between edges
     if(edge_falling > edge_rising){
       dis = ((overflow*65536 + (edge_falling - edge_rising))/24)-100;
       if(dis<3000){
          distance = dis;
       }
       
     }else if(edge_falling < edge_rising){
       dis = ((overflow*65536 - (edge_rising - edge_falling))/24)-100;
     
       if(dis<3000){
          distance = dis;
       }
     
     }
  }
}



// !!!!!!!!Hongzhan Yu modified in 24/05/2018
//This interrupt calculate the times timer overflow
interrupt 16 void Overflow_ISR(void){
   TFLG2 = TFLG2 | TFLG2_TOF_MASK;
   overflow++;
}


//TC5 getting values from gyro 
 
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
 
   