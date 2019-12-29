/*
 * File:        sword.c (Kendo Sword Trainer with Dummy) 
 * 
 * Author:      Iman Nandi and Weichen Zhou
 * For use with Sean Carroll's Big Board
 * http://people.ece.cornell.edu/land/courses/ece4760/PIC32/target_board.html
 * Target PIC:  PIC32MX250F128B
 * IDE: MPLAB X IDE v3.05 or v5.25
 * Compiler: XC32 v1.40
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_3_2.h"
// threading library
#include "pt_cornell_1_3_2.h"

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"

// I2C 2-wire connections to LSM9DS1 (Reference: https://github.com/guiklink/Sparkfun_LSM9DS1_PIC32)
#include "SparkFun_LSM9DS1.h"
#include "LSM9DS1_Registers.h"

// need for rand function
#include <stdlib.h>
// need for sin function
#include <math.h>
#include <plib.h>

// lock out timer 2 interrupt during spi communication to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)

// === the fixed point macros ========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)

// LSM9DS1 sensor variables for receiving feedback over I2C connection
int lsmSetup = 0;
signed short int gyro[3], accel[3], magn[3];

// Direction calculation variables
_Accum gForce;
_Accum gForceOld;
_Accum gForceNormalized[3];
_Accum directionEstimate[3];
_Accum directionEstimateOld[3];
_Accum aXZ;
_Accum aXZOld;
_Accum aYZ;
_Accum aYZOld;
_Accum RGyro[3];
_Accum signDirectionEstimateZ;
_Accum directionFinal[3];
_Accum directionFinalNormalized;
_Accum directionVal[3];
_Accum directionSample[5][3];
_Accum weightGyro = 10; // Between 5 and 20 for calculating direction accurately
unsigned int directionSampleSize = 5; // Change based on size of directionSample array below
unsigned int directionSampleScore = 0;

// Flags for identifying when events occurred
unsigned int hitStart = 0; 
unsigned int hitFlash = 0; 
unsigned int hitComplete = 0;

// Variables for delays between TFT display events
int hitTime = -5;
int potentialMissTime = 0;
unsigned int potentialMissed = 0;
unsigned int potentialMissEndTime = 7;

unsigned int hitCounter = 0; // Keeps track of number of hits to helmet

// Variables for scoring system and feedback
unsigned int rating;
char* ratingComment = "";

// Variables for threading
static struct pt pt_timer, pt_adc, pt_lsm, pt_tft ;

//ADC Variables
int adc_0, adc_1, adc_5;
float V_0, V_5, V_11;
int adc_counter0 = 0;
int adc_counter1 = 0;
int adc_counter5 = 0;
int sample_counter = 0;
int adcThresh = 50;
int adc_max_0 = 0;
int adc_max_1 = 0;
int adc_max_5 = 0;

// string buffer
char buffer[60];

// Keeps track of time elapsed in timer thread
int sys_time_seconds ;

// === LSM9DS1 (Gyroscope and Accelerometer) Thread =================================================
static PT_THREAD (protothread_lsm(struct pt *pt))
{
    PT_BEGIN(pt);
    static int lsmTime = 20;
    static int i, j;
    
    while(1) {
        PT_YIELD_TIME_msec(lsmTime);
        
        tft_fillRoundRect(0,210,320,60,1,ILI9340_BLACK); // Clear draw line section for LSM9DS1 direction display of sword 

        // Get new values for accelerometer and gyroscope values
        get_accel(accel);
        get_gyro(gyro);
        
        // Compute direction with both gyro and accelerometer
        gForceOld = gForce;
        gForce = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
        for(i=0; i<3; i++) {
            gForceNormalized[i] = accel[i] / gForce;
        }
        
        for(i=0; i<3; i++) {
            directionEstimate[i] = accel[i];
        }
        
        aXZOld = atan2(directionEstimateOld[0], directionEstimateOld[2]);
        aXZ = aXZOld + (gyro[1]+20)*0.00875/6.28 * (lsmTime/1000.0);
        
        aYZOld = atan2(directionEstimateOld[1], directionEstimateOld[2]);
        aYZ = aYZOld + (gyro[0]-390)*0.00875/6.28 * (lsmTime/1000.0);
        
        RGyro[0] = sin(aXZ) / sqrt(1 + cos(aXZ)*cos(aXZ)*tan(aYZ)*tan(aYZ));
        RGyro[1] = sin(aYZ) / sqrt(1 + cos(aYZ)*cos(aYZ)*tan(aXZ)*tan(aXZ));
        
        if(directionEstimateOld[2]>=0) signDirectionEstimateZ = 1;
        else signDirectionEstimateZ = -1;
                
        RGyro[2] = signDirectionEstimateZ * sqrt(1 - RGyro[0]*RGyro[0] - RGyro[1]*RGyro[1]);
                
        for(i=0; i<3; i++) {
            directionFinal[i] = (gForceNormalized[i] + RGyro[i] * weightGyro) / (1 + weightGyro);
        }
        
        directionFinalNormalized = sqrt(directionFinal[0]*directionFinal[0] + directionFinal[1]*directionFinal[1] + directionFinal[2]*directionFinal[2]);
        for(i=0; i<3; i++) {
            directionVal[i] = directionFinal[i] / directionFinalNormalized;
        }
        
        aXZOld = aXZ;    
        aYZOld = aYZ;
        for(i=0; i<3; i++) {
            directionEstimateOld[i] = directionEstimate[i];
        }           
        
        // Pick up a sample every cycle
        if(hitStart && !hitComplete) {
            // Store samples of direction with sliding window
            for (i=0; i<directionSampleSize-1; i++) // Sample number
            {
                for (j=0; j<3; j++) // X, Y, and Z
                {
                    directionSample[i][j] = directionSample[i+1][j];;
                }
            }
            for (j=0; j<3; j++) // Get new sample at end of window
            {
                directionSample[directionSampleSize-1][j] = directionVal[j];
            }
        }
        
        if(abs(gForce - gForceOld) > 10000 && !hitStart) // Indicator that it started
        {
            hitStart = 1; // This will reset to 0 after rating determined
            adc_max_0 = 0;
            adc_max_1 = 0;
            adc_max_5 = 0;
            adc_counter0 = 0;
            adc_counter1 = 0;
            adc_counter5 = 0;
            sample_counter =0;
            
            if(!potentialMissed) {
                potentialMissTime = sys_time_seconds;
                potentialMissed = 1;
            }
            
            hitFlash = 1;
        }
        
        // Check which ADC counter is the smallest to determine which side of the helmet was hit. 
        // This section will occur NEXT cycle after hitComplete flag is triggered
        if((((adc_counter5-14) < adc_counter1 && (adc_counter5-14) < adc_counter0) || abs(adc_counter1-adc_counter0) == 100) && hitComplete) {
            ratingComment = "Right on target!";
            hitCounter++; // Increase hit counter
            
            // Check if directionSample is good
            for (i=0; i<directionSampleSize; i++) // Sample number
            {
                    if (fabs((float)directionSample[i][1]) <= 0.3) directionSampleScore++;
            }
            if(directionSampleScore > (directionSampleSize - 2)) rating = 5;
            else if(directionSampleScore > (directionSampleSize - 3)) rating = 4;
            else rating = 3;
            directionSampleScore = 0; // Reset directionSampleCounter for next scoring
            hitStart = 0;
            hitComplete = 0;
            potentialMissed = 0;
        }
        else if (hitComplete) {
            // Check which side you hit on
            if (adc_counter0 < adc_counter1 && adc_counter0 < adc_counter5) ratingComment = "Too far left";
            else if (adc_counter1 < adc_counter0 && adc_counter1 < adc_counter5) ratingComment = "Too far right";
            hitCounter++; // Increase hit counter
            
            // Check if directionSample is good
            for (i=0; i<directionSampleSize; i++) // Sample number
            {
                if (fabs((float)directionSample[i][1]) <= 0.3) directionSampleScore++;
            }
            if(directionSampleScore > (directionSampleSize - 2)) rating = 2;
            else if(directionSampleScore > (directionSampleSize - 3)) rating = 1;
            else rating = 0;
            directionSampleScore = 0; // Reset directionSampleCounter for next scoring
            hitStart = 0;
            hitComplete = 0;
            potentialMissed = 0;
        }
        
        // Check ADC reading on helmet passed thresholds
        if((adc_max_0 > 10 || adc_max_5 > 22 || adc_max_1 > 10) && !hitComplete && hitStart) 
        {
            hitComplete = 1; // This will reset to 0 after rating is determined
            hitTime = sys_time_seconds; // Record time of hit
        }
        
        tft_drawLine( 120,240, (int)(120+directionVal[0]*30),(int)(240+directionVal[1]*30),ILI9340_BLUE);
        
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
}

// === ADC Thread =============================================
static PT_THREAD (protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);
            
    while(1) {
        // yield time 1 second
        PT_YIELD(pt);
        
        //read selected ANx pins in order, AN0 first
        adc_0 = ReadADC10(0);   // AN0 (RA0) 
        adc_1 = ReadADC10(1);   // AN1 (RA1)
        adc_5 = ReadADC10(2);   // AN5 (RB2)
        
        //sample counter to see  sample rate/s
        sample_counter++;
        
        // Detect ADC peaks
        if(adc_max_0 < adc_0 && hitStart)
        {
            adc_max_0 = adc_0;
            adc_counter0 = sample_counter;
        }
        
        if(adc_max_1 < adc_1 && hitStart)
        {
            adc_max_1 = adc_1;
            adc_counter1 = sample_counter;
        }
        
        if(adc_max_5 < adc_5 && hitStart)
        {
            adc_max_5 = adc_5;
            adc_counter5 = sample_counter;
        }
        
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // animation thread

// === print a line on TFT =====================================================
// Utilities to print a line on the TFT
// Predefined colors definitions (from tft_master.h)
//#define	ILI9340_BLACK   0x0000
//#define	ILI9340_BLUE    0x001F
//#define	ILI9340_RED     0xF800
//#define	ILI9340_GREEN   0x07E0
//#define ILI9340_CYAN    0x07FF
//#define ILI9340_MAGENTA 0xF81F
//#define ILI9340_YELLOW  0xFFE0
//#define ILI9340_WHITE   0xFFFF

// === TFT LCD Thread =================================================
static PT_THREAD (protothread_tft(struct pt *pt))
{
    PT_BEGIN(pt);
    
    while(1) {
        PT_YIELD_TIME_msec(1000);
        
        // Flash screen when a hit has occurred
        if(hitFlash) {
            tft_fillScreen(ILI9340_RED);
            tft_fillScreen(ILI9340_BLACK);
            hitFlash = 0;
        }
        
        tft_fillRoundRect(3,285, 230, 15, 1, ILI9340_BLACK); // Clear DEBUG line
        tft_fillRoundRect(3,300, 230, 15, 1, ILI9340_BLACK); // Clear DEBUG line
        
        // Clear TFT display lines as necessary for design
        tft_fillRoundRect(5,30, 240, 15, 1, ILI9340_BLACK);
        tft_fillRoundRect(3,70, 230, 15, 1, ILI9340_BLACK);
        tft_fillRoundRect(3,90, 230, 15, 1, ILI9340_BLACK);
        tft_fillRoundRect(3,130, 230, 15, 1, ILI9340_BLACK);
      
        // TITLE
        tft_setCursor(10, 0);
        tft_setTextSize(3);
        tft_setTextColor(ILI9340_BLUE); 
        sprintf(buffer,"KENDO TRAINER");
        tft_writeString(buffer);
        
        if(hitStart && !hitComplete && (sys_time_seconds - potentialMissTime) > 3 && (sys_time_seconds - potentialMissTime) < potentialMissEndTime) {
            tft_setCursor(5, 30);
            tft_setTextSize(2);
            tft_setTextColor(ILI9340_RED); 
            sprintf(buffer,"You missed! Focus!");
            tft_writeString(buffer);
            
            tft_setCursor(3, 90);
            tft_setTextSize(2);
            tft_setTextColor(ILI9340_GREEN); 
            sprintf(buffer,"Score: TBD");
            tft_writeString(buffer);
        }
        else if((sys_time_seconds - hitTime) < 5) {
            tft_setCursor(5, 30);
            tft_setTextSize(2);
            tft_setTextColor(ILI9340_GREEN); 
            sprintf(buffer,"Hit! See your score!");
            tft_writeString(buffer);
            
            tft_setCursor(3, 90);
            tft_setTextSize(2);
            tft_setTextColor(ILI9340_GREEN); 
            sprintf(buffer,"Score: %d", rating);
            tft_writeString(buffer);
        }
        else {
            tft_setCursor(5, 30);
            tft_setTextSize(2);
            tft_setTextColor(ILI9340_YELLOW); 
            sprintf(buffer,"Waiting for hit...");
            tft_writeString(buffer);
            
            tft_setCursor(3, 90);
            tft_setTextSize(2);
            tft_setTextColor(ILI9340_GREEN); 
            sprintf(buffer,"Score: TBD");
            tft_writeString(buffer);
        }
        
        if((sys_time_seconds - potentialMissTime) == potentialMissEndTime) {
            potentialMissed = 0; // Reset potential miss flag
            hitStart = 0;
        }
        
        tft_setCursor(3, 70);
        tft_setTextSize(2);
        tft_setTextColor(ILI9340_GREEN); 
        sprintf(buffer,"Hit Count: %d", hitCounter);
        tft_writeString(buffer);
        
        tft_setCursor(3, 130);
        tft_setTextSize(2);
        tft_setTextColor(ILI9340_YELLOW); 
        sprintf(buffer,"%s", ratingComment);
        tft_writeString(buffer);
        
        // Display direction values that are calculated in lsm thread
        tft_setCursor(3, 285);
        tft_setTextSize(1);
        tft_setTextColor(ILI9340_YELLOW); 
        sprintf(buffer,"DIR: %f %f %f", (float)directionVal[0], (float)directionVal[1],(float)directionVal[2]);
        tft_writeString(buffer);
        
        // Display ADC values of counter to see which ADC counter is the smallest when hit
        // Helps to identify how a certain helmet area was identified as being hit
        tft_setCursor(3, 300);
        tft_setTextSize(1);
        tft_setTextColor(ILI9340_YELLOW); 
        sprintf(buffer,"ADC: L-%d M-%d R-%d", adc_counter0, adc_counter5, adc_counter1);
        tft_writeString(buffer);
        
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} 

// === Timer Thread =================================================
// system 1 second interval tick
// prints on TFT 
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt); 
    
    while(1){
        PT_YIELD_TIME_msec(1000) ;
        tft_fillRoundRect(3, 270, 230, 15, 1, ILI9340_BLACK);
        sys_time_seconds++;
        tft_setCursor(3, 270);
        tft_setTextColor(ILI9340_YELLOW); 
        tft_setTextSize(1);
        sprintf(buffer, "Time=%d", sys_time_seconds);
        tft_writeString(buffer);
        
        // NEVER exit while
    } // END WHILE(1)
     
  PT_END(pt);
} // timer thread

// === Main  ======================================================
void main(void) {
    //SYSTEMConfigPerformance(PBCLK);
  
    ANSELA = 0; ANSELB = 0; 
  
    // set up DAC on big board
    // timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // at 40 MHz PB clock 
    // 40,000,000/Fs = 909 : since timer is zero-based, set to 908
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 600);

    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // clk divider set to 4 for 10 MHz
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL, 2);
    // end DAC setup
    
    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(4, RPB10, SS2);
    
    PPSOutput(2, RPB5, SDO2);

    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();
  
    // the ADC ///////////////////////////////////////
    // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 3 samples | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_3 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF

	// Define setup parameters for OpenADC10
    
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy 
    
	// set AN0 (RA0 / left side), AN1 (RA1 / middle side), AN0 (RB3 / right side) as analog inputs
	#define PARAM4	ENABLE_AN5_ANA | ENABLE_AN1_ANA | ENABLE_AN0_ANA

    // Do not skip the channels you want to scan
	#define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
	
    // use ground as neg ref for A 
    // actual channel number is specified by the scan list
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); // 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
    ///////////////////////////////////////////////////////
    
    // init the threads
    PT_INIT(&pt_timer);
    PT_INIT(&pt_adc);
    PT_INIT(&pt_tft);
    PT_INIT(&pt_lsm);
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  
    // LSM9DS1 (Gyroscope and Accelerometer) setup
    lsmSetup = i2c_master_setup();
    config_gyro_accel_default();
    
    // round robin scheduler for threads
    while(1) {
        PT_SCHEDULE(protothread_timer(&pt_timer));
        PT_SCHEDULE(protothread_adc(&pt_adc));
        PT_SCHEDULE(protothread_tft(&pt_tft));
        PT_SCHEDULE(protothread_lsm(&pt_lsm));
    }
  
} // main

// === end  ======================================================

