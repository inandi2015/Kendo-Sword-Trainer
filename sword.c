/*
 * File:        sword.c (Kendo Sword Trainer with Dummy) 
 * 
 * Author:      Iman Nandi and Weichen Zhou
 * For use with Sean Carroll's Big Board
 * http://people.ece.cornell.edu/land/courses/ece4760/PIC32/target_board.html
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_3_2.h"
// threading library
#include "pt_cornell_1_3_2.h"
// yup, the expander
#include "port_expander_brl4.h"

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
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

////////////////////////////////////
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

int gyroSetup = 0;
int sthelse = 3;
signed short int gyro[3], accel[3], magn[3];
signed short int gyro_max[3]= {0, 0, 0} ;
signed short int gyro_min[3]= {0, 0, 0} ;
int gyro_sample[5][3];
int gyro_avg[3];
int gyro_record[3];
int timer;
int direction[3] = {0, 0, 0} ;

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
//_Accum gyroOld[3];
_Accum RGyro[3];
_Accum signDirectionEstimateZ;
_Accum directionFinal[3];
_Accum weightGyro = 10; // Between 5 and 20
_Accum directionFinalNormalized;
_Accum directionVal[3];

unsigned int directionSampleSize = 5; // Change based on size of directionSample array below
_Accum directionSample[5][3];
unsigned int directionSampleScore = 0;
unsigned int hitStart = 0; 
unsigned int hitStartFlag = 0; 
unsigned int hitComplete = 0;
unsigned int xHigh = 0;
int hitTime = -5;
int potentialMissTime = 0;
unsigned int hitCounter = 0;
unsigned int potentialMissed = 0;
unsigned int potentialMissEndTime = 7;

unsigned int rating;
char* ratingComment = "";

static struct pt pt_timer, pt_lsm, pt_tft ;
static struct pt pt_timer, pt_adc, pt_lsm, pt_direction, pt_tft ;

//ADC Variables
int adc_0, adc_1, adc_5;
float V_0, V_5, V_11;
int adc_counter0 = 0;
int adc_counter1 = 0;
int adc_counter5 = 0;
int sample_counter = 0;
int adcThresh = 50;

//int adc_flag_0 = 0;
//int adc_flag_1 = 0;
//int adc_flag_5 = 0;
//int adc_flag_peak_0 = 0;
//int adc_flag_peak_1 = 0;
//int adc_flag_peak_5 = 0;
//int adc_temp_0 = 0;
//int adc_temp_1 = 0;
//int adc_temp_5 = 0;

int adc_max_0 = 0;
int adc_max_1 = 0;
int adc_max_5 = 0;

int sys_time_seconds ;
// thread identifier to set thread parameters
int thread_num_timer;

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

// string buffer
char buffer[60];
void printLine(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 10 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 8, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(1);
    tft_writeString(print_buffer);
}

void printLine2(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 20 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 16, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(2);
    tft_writeString(print_buffer);
}

static PT_THREAD (protothread_lsm(struct pt *pt))
{
    PT_BEGIN(pt);
    static int lsmTime = 20;
    static int i, j;
    while(1) {
        PT_YIELD_TIME_msec(lsmTime);
        
        tft_fillRoundRect(0,210,320,60,1,ILI9340_BLACK); // Clear draw line section for gyro display of sword 
        
//        tft_fillRoundRect(3,130, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line 
//        tft_fillRoundRect(3,150, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
//        tft_fillRoundRect(3,170, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
//        tft_fillRoundRect(3,190, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
//        tft_fillRoundRect(3,210, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
//        tft_fillRoundRect(3,230, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
        
        //sprintf(buffer,"Something8");
        //printLine2(8, buffer, ILI9340_BLACK, ILI9340_YELLOW);
        get_accel(accel);
        get_gyro(gyro);
//        direction[0]=((gyro[0]-390)/64)+accel[0];
//        direction[1]=(gyro[1]+20)/64+accel[1];
//        direction[2]=gyro[2]/64+accel[2];
        direction[0]=accel[0];
        direction[1]=accel[1];
        direction[2]=accel[2];
        
//        if(gyro_max[0]<gyro[0]) gyro_max[0] = gyro[0];
//        if(gyro_min[0]>gyro[0]) gyro_min[0] = gyro[0];
//        if(gyro_max[1]<gyro[1]) gyro_max[1] = gyro[1];
//        if(gyro_min[1]>gyro[1]) gyro_min[1] = gyro[1];
//        if(gyro_max[2]<gyro[2]) gyro_max[2] = gyro[2];
//        if(gyro_min[2]>gyro[2]) gyro_min[2] = gyro[2];
        
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
//        for(i=0; i<3; i++) {
//            gyroOld[i] = gyro[i];
//        }       
        
        
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
            
            hitStartFlag = 1;
        }
        
        // This will occur NEXT cycle after hitComplete flag is triggered
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
        
        /* Logic for what's a great rating (ADC 5 is the middle) */
        // Middle piezoelectric response was smallest 
        // CHANGE adcCounter threshold for hit --> i.e. the hitThreshold
//        if((adc_max_5+adc_max_1+adc_max_0) > hitThreshold && !hitComplete && hitStart || ) 
//        {
//            hitComplete = 1; // This will reset to 0 after display
//            adc_counter0 = 0;
//            adc_counter1 = 0;
//            adc_counter5 = 0;
//            adc_max_0 = 0;
//            adc_max_1 = 0;
//            adc_max_5 = 0;
//        }
        if((adc_max_0 > 10 || adc_max_5 > 22 || adc_max_1 > 10) && !hitComplete && hitStart) 
        {
            hitComplete = 1; // This will reset to 0 after rating determined
            hitTime = sys_time_seconds; // Record time of hit
            
        }
        
        
        tft_drawLine( 120,240, (int)(120+directionVal[0]*30),(int)(240+directionVal[1]*30),ILI9340_BLUE);
        
//               // DEBUG
//        tft_setCursor(3, 150);
//        tft_setTextSize(1);
////        sprintf(buffer,"DEBUG: %d", hitComplete);
//        sprintf(buffer,"DEBUG: %f %f %f", (float)directionSample[0][0], fabs((float)directionSample[0][1]),(float)directionSample[0][2]);
//        tft_writeString(buffer);
//        
//                // DEBUG
//        tft_setCursor(3, 170);
//        tft_setTextSize(1);
////        sprintf(buffer,"DEBUG: %d", hitComplete);
//        sprintf(buffer,"DEBUG: %f %f %f", (float)directionSample[1][0], fabs((float)directionSample[1][1]),(float)directionSample[1][2]);
//        tft_writeString(buffer);
//        
//                // DEBUG
//        tft_setCursor(3, 190);
//        tft_setTextSize(1);
////        sprintf(buffer,"DEBUG: %d", hitComplete);
//        sprintf(buffer,"DEBUG: %f %f %f", (float)directionSample[2][0], fabs((float)directionSample[2][1]),(float)directionSample[2][2]);
//        tft_writeString(buffer);
//        
//                // DEBUG
//        tft_setCursor(3, 210);
//        tft_setTextSize(1);
////        sprintf(buffer,"DEBUG: %d", hitComplete);
//        sprintf(buffer,"DEBUG: %f %f %f", (float)directionSample[3][0], fabs((float)directionSample[3][1]),(float)directionSample[3][2]);
//        tft_writeString(buffer);
//        
//                // DEBUG
//        tft_setCursor(3, 230);
//        tft_setTextSize(1);
////        sprintf(buffer,"DEBUG: %d", hitComplete);
//        sprintf(buffer,"DEBUG: %f %f %f", (float)directionSample[4][0], fabs((float)directionSample[4][1]),(float)directionSample[4][2]);
//        tft_writeString(buffer);
        
    }
    PT_END(pt);
}

// update a 1 second tick counter
static PT_THREAD (protothread_direction(struct pt *pt))
{
    PT_BEGIN(pt);
    int i, j;
        while(1)
        {
            PT_YIELD(pt);
            
            // Slide window of gyro samples over
            for (i=0; i<4; i++) // Sample number
            {
                for (j=0; j<3; j++) // X, Y, or Z
                {
                    gyro_sample[i][j] = gyro_sample[i+1][j];
                }
            }
            for (j=0; j<3; j++) // Get new sample at end of window
            {
                if(j ==0)
                {
                    gyro_sample[4][j] = gyro[j]-420;
                }
                if(j==1)
                {
                    gyro_sample[4][j] = gyro[j]+90;
                }
                if(j==2)
                {
                    gyro_sample[4][j] = gyro[j]+30;
                
                }    
            }
                
            
                
            for (j=0; j<3; j++) // X, Y, or Z
            {
                gyro_avg[j] = 0;
            }
            // Get gyro avg
            for (i=0;i<5;i++) // Sample number
            {
                for (j=0; j<3; j++) // X, Y, or Z
                {
                    gyro_avg[j] += gyro_sample[i][j];
                }
            }
            
//            for (j=0;j<3;j++)
//            {
//                if((gyro_avg[j])>32768)
//                {
//                    gyro_record[j] =gyro_avg[j]-65536;
//                }
//                else
//                {
//                    gyro_record[j] = gyro_avg[j];
//                }
//            }
            
            for (j=0;j<3;j++)
            {
                direction[j] += gyro_avg[j]/160;
            }
            
        }
    PT_END(pt);
}

// === ADC Thread =============================================
// 

static PT_THREAD (protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);
    static int i;
            
    while(1) {
        // yield time 1 second
        PT_YIELD(pt);
        
        //read selected ANx pins in order, AN5 first
        
        
        adc_0 = ReadADC10(0);   // 
        
        adc_1 = ReadADC10(1);   // 
       
        // then AN11
        adc_5 = ReadADC10(2);
        
        //sample counter to see  sample rate/s
        sample_counter++;
        
        //AcquireADC10(); 
        
        // Record sample counter to compare which ADC reached peak value to tell which piezoelectric sensor received a value first
//        if (adc_0 > adcThresh && !adc_flag_0)
//        {
//            //adc_counter0 = sample_counter;
//            adc_flag_0 = 1;
//        }
//       
//        if (adc_1 > adcThresh && !adc_flag_1)
//        {
//         //   adc_counter1 = sample_counter;
//            adc_flag_1 = 1;
//        }
//        if (adc_5 > adcThresh && !adc_flag_5)
//        {
//          // adc_counter5 = sample_counter;
//            adc_flag_5 = 1;
//        }
        
//         if(adc_flag_0 && adc_0 > adc_temp_0 && !adc_flag_peak_0)
//        {
//            adc_counter0 = sample_counter;
//            adc_flag_peak_0 = 1;
//            adc_max_0 = adc_0;
//        }
//         if(adc_flag_1 && adc_1 < adc_temp_1&& !adc_flag_peak_1)
//        {
//            adc_counter1 = sample_counter;
//            adc_flag_peak_1 = 1;
//        }
//         if(adc_flag_5 && adc_5 < adc_temp_5&& !adc_flag_peak_5)
//        {
//            adc_counter5 = sample_counter;
//            adc_flag_peak_5 = 1;
//        }
//        
        
//        adc_temp_0 = adc_0;
//        adc_temp_1 = adc_1;
//        adc_temp_5 = adc_5;
        
        // Test: another way to detect peak:
        if(adc_max_0 < adc_0 && hitStart)
        {
            adc_max_0 = adc_0;
            adc_counter0 = sample_counter;
        }
        
        if(adc_max_1 < adc_1 &&hitStart)
        {
            adc_max_1 = adc_1;
            adc_counter1 = sample_counter;
        }
        
        if(adc_max_5 < adc_5 &&hitStart)
        {
            adc_max_5 = adc_5;
            adc_counter5 = sample_counter;
        }
        
//        tft_fillRoundRect(3,110, 230, 15, 1, ILI9340_BLACK);
//        // DEBUG
//        tft_setCursor(3, 110);
//        tft_setTextSize(2);
//        tft_setTextColor(ILI9340_RED); 
//        sprintf(buffer,"HS: %d HC: %d HT: %d", hitStart, hitComplete, hitTime);
//        tft_writeString(buffer);
        
        
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // animation thread


static PT_THREAD (protothread_tft(struct pt *pt))
{
    PT_BEGIN(pt);
    
            
    while(1) {
        PT_YIELD_TIME_msec(1000);
        
        if(hitStartFlag) {
            tft_fillScreen(ILI9340_RED);
            tft_fillScreen(ILI9340_BLACK);
            hitStartFlag = 0;
        }
        
        tft_fillRoundRect(3,285, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
        tft_fillRoundRect(3,300, 230, 15, 1, ILI9340_BLACK);// Clear DEBUG line
        
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
        
                
        // DEBUG
        tft_setCursor(3, 285);
        tft_setTextSize(1);
        tft_setTextColor(ILI9340_YELLOW); 
//        sprintf(buffer,"DEBUG: %d", hitComplete);
        sprintf(buffer,"DIR: %f %f %f", (float)directionVal[0], (float)directionVal[1],(float)directionVal[2]);
        tft_writeString(buffer);
        
        // DEBUG
        tft_setCursor(3, 300);
        tft_setTextSize(1);
        tft_setTextColor(ILI9340_YELLOW); 
//        sprintf(buffer,"DEBUG: %d", hitComplete);
        sprintf(buffer,"ADC: L-%d M-%d R-%d", adc_counter0, adc_counter5, adc_counter1);
        tft_writeString(buffer);
        
//         //AN0
//         //draw adc and voltage
//        tft_fillRoundRect(0,10, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%d", adc_max_0);
//        tft_writeString(buffer);
//        
//         //AN5
//         //draw adc and voltage
//        tft_fillRoundRect(0,50, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 50);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%d", adc_max_1);
//        tft_writeString(buffer);
//        
//        //AN11
//        // draw adc and voltage
//        tft_fillRoundRect(0,100, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 100);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//            sprintf(buffer,"%d", adc_max_5);
//        tft_writeString(buffer);
        
        
                
//        tft_fillRoundRect(3,120, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//
//        tft_setCursor(3, 120);
//        // print raw ADC, floating voltage, fixed voltage
//        sprintf(buffer,"Counter 0: %d", adc_counter0);
//        tft_writeString(buffer);
//        
//        tft_fillRoundRect(3,140, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//
//        tft_setCursor(3, 140);
//        // print raw ADC, floating voltage, fixed voltage
//        sprintf(buffer,"Counter 1: %d", adc_counter1);
//        tft_writeString(buffer);
//        
//        tft_fillRoundRect(3,160, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//
//        tft_setCursor(3, 160);
//        // print raw ADC, floating voltage, fixed voltage
//        sprintf(buffer,"Counter 5: %d", adc_counter5);
//        tft_writeString(buffer);
//        sample_counter= 0;
        
        
//        tft_drawLine( 30,240, (int)(30+directionEstimate[0]/320),(int)(240+directionEstimate[1]/320),ILI9340_YELLOW);
//        tft_drawLine( 90,240, 90+gForceNormalized[0]*30,240+gForceNormalized[1]*30,ILI9340_YELLOW);
        //tft_drawLine( 150,240, (int)(150+directionVal[0]*30),270,ILI9340_YELLOW);
       
        
//        // DEBUG
//        tft_setCursor(3, 130);
//        tft_setTextSize(1);
////        sprintf(buffer,"DEBUG: %d", hitComplete);
//        sprintf(buffer,"DEBUG GFORCE: %f %f %f", (float)gForceNormalized[0], (float)gForceNormalized[1],(float)gForceNormalized[2]);
//        tft_writeString(buffer);
//     
        
        
 
        
        //        tft_drawLine( 30-direction[1]/640,240-direction[2]/640, 60+direction[1]/640,240+direction[2]/640,ILI9340_YELLOW);
//        tft_drawLine( 90-direction[0]/640,240-direction[1]/640, 90+direction[0]/640,240+direction[1]/640,ILI9340_YELLOW);
//        tft_drawLine(150-direction[2]/640,240-direction[0]/640,150+direction[2]/640,240+direction[0]/640,ILI9340_YELLOW);
        
        
        
//        tft_fillRoundRect(3,200, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        sprintf(buffer, "AX = %d | AY = %d | AZ = %d", accel[0],accel[1],accel[2]);
//        tft_setCursor(10, 200);
//        tft_setTextColor(ILI9340_YELLOW); 
//        tft_setTextSize(1);
//        tft_writeString(buffer);
//
//        tft_fillRoundRect(3,250, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        sprintf(buffer, "GX = %d | GY = %d | GZ = %d", gyro_avg[0],gyro_avg[1],gyro_avg[2]);
//        tft_setCursor(3, 250);
//        tft_setTextColor(ILI9340_YELLOW); 
//        tft_setTextSize(1);
//        tft_writeString(buffer);
//        
//        
//        tft_fillRoundRect(3,300, 230, 15, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        sprintf(buffer, "d_X = %d | d_Y = %d | d_Z = %d", direction[0],direction[1],direction[2]);
//        tft_setCursor(3, 300);
//        tft_setTextColor(ILI9340_YELLOW); 
//        tft_setTextSize(1);
//        tft_writeString(buffer);
        
        
    //    get_mag(magn);
    //    sprintf(buffer, "MX = %d | MY = %d | MZ = %d", magn[0],magn[1],magn[2]);
    //    tft_setCursor(10, 200);
    //    tft_setTextColor(ILI9340_YELLOW); 
    //    tft_setTextSize(1);
    //    tft_writeString(buffer);
        
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} 

// === Timer Thread =================================================
// system 1 second interval tick
// prints on TFT and blinks LED on RA0

static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt); 
    while(1){
        // yield time 1 second
//        PT_YIELD_TIME_msec(66) ;
//        // draw sys_time  
//        //sys_time_seconds++;
//        tft_setCursor(1, 290);
//        tft_setTextColor(ILI9340_YELLOW); 
//        tft_setTextSize(1);
//        
//        sprintf(buffer,"%d", gyroSetup);
//        tft_writeString(buffer);
//        while(1){
            PT_YIELD_TIME_msec(1000) ;
            tft_fillRoundRect(3, 270, 230, 15, 1, ILI9340_BLACK);
            sys_time_seconds++;
            tft_setCursor(3, 270);
            tft_setTextColor(ILI9340_YELLOW); 
            tft_setTextSize(1);
            sprintf(buffer, "Time=%d", sys_time_seconds);
            tft_writeString(buffer);
        
          
          
        
//        }
    //     tft_setCursor(30, 290);
    //      tft_setTextColor(ILI9340_YELLOW); 
    //      tft_setTextSize(1);
    //      sprintf(buffer, "counter343434 = %d", counter);
    //      tft_writeString(buffer);
    } // END WHILE(1)
     
//        sprintf(buffer,"Something");
//    printLine2(4, buffer, ILI9340_BLACK, ILI9340_YELLOW);
//      while(1) {
//          sprintf(buffer,"Something5");
//    printLine2(5, buffer, ILI9340_BLACK, ILI9340_YELLOW);
//        // yield time 1 second
//        PT_YIELD_TIME_msec(1000) ;
//        
//          sprintf(buffer,"Something6");
//    printLine2(6, buffer, ILI9340_BLACK, ILI9340_YELLOW);
//        sys_time_seconds++ ;
////        // toggle the LED on the big board
////        mPORTAToggleBits(BIT_0);
//        // draw sys_time
//        sprintf(buffer,"Time=%d", sys_time_seconds);
//        printLine2(0, buffer, ILI9340_BLACK, ILI9340_YELLOW);
//        // NEVER exit while
//      } // END WHILE(1)
     
  PT_END(pt);
} // timer thread

// === Main  ======================================================
void main(void) {
    //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 
  //setup reset button
  
  
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
        //
	// Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy 

	// define setup parameters for OpenADC10
	// set AN0 (RA0 / left side), AN1 (RA1 / middle side), AN0 (RB3 / right side) as analog inputs
	#define PARAM4	ENABLE_AN5_ANA | ENABLE_AN1_ANA | ENABLE_AN0_ANA
//#define PARAM4 ENABLE_AN12_ANA | ENABLE_AN11_ANA | ENABLE_AN10_ANA | ENABLE_AN9_ANA | ENABLE_AN8_ANA | ENABLE_AN7_ANA | ENABLE_AN6_ANA | ENABLE_AN5_ANA | ENABLE_AN4_ANA | ENABLE_AN3_ANA | ENABLE_AN2_ANA | ENABLE_AN1_ANA | ENABLE_AN0_ANA
	// define setup parameters for OpenADC10
    // DO not skip the channels you want to scan
    // do not specify channels 0, 1, and 5
	#define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
//#define PARAM5 SKIP_SCAN_ALL
	// use ground as neg ref for A 
    // actual channel number i	s specified by the scan list
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); // 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
  ///////////////////////////////////////////////////////
    
  // init the threads
  PT_INIT(&pt_adc);
  PT_INIT(&pt_timer);
  PT_INIT(&pt_lsm);
  PT_INIT(&pt_direction);
  PT_INIT(&pt_tft);
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  
        // Gyroscope setup (RED/BLUE WIRE - Vdd, BLACK/GRAY WIRE - Gnd, WHITE WHIRE - SCL, GREEN/PURPLE WIRE - SDA)
        gyroSetup = i2c_master_setup();
        config_gyro_accel_default();
        
//        sprintf(buffer,"Something");
//        printLine2(2, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    
    // round robin scheduler for threads
    while(1) {
        PT_SCHEDULE(protothread_timer(&pt_timer));
        PT_SCHEDULE(protothread_adc(&pt_adc));
        PT_SCHEDULE(protothread_tft(&pt_tft));
        PT_SCHEDULE(protothread_lsm(&pt_lsm));
        //PT_SCHEDULE(protothread_direction(&pt_direction)); // SEE POSE project 
    }
  
} // main

// === end  ======================================================

