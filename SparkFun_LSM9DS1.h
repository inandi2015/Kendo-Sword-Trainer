/* 
 * File:   i2c_master_int.h
 * Author: klink
 *
 * Created on April 12, 2015, 4:41 PM
 */


#ifndef SPARKFUNLSM9DS1__H__
#define SPARKFUNLSM9DS1__H__

//////////////////////////////////////
// Pre-defined scales for gyroscope //
//////////////////////////////////////
#define G_SCALE_1 245 / 32768.0
#define G_SCALE_2 500.0 / 32768.0
#define G_SCALE_3 2000.0 / 32768.0


///////////////////////////////////////////
// Slave addresses for SDO set as 0 or 1 //
///////////////////////////////////////////
#define SAD_AG_0 0x6A
#define SAD_AG_1 0x6B
#define SAD_M_0 0x1C
#define SAD_M_1 0x1E





unsigned char test_A_G();					// ping WHO_AM_I register from accel and gyro
unsigned char test_M();						// ping WHO_AM_I register from magnetometer


int i2c_master_setup(void);              	// set up I2C 1 as a master, at 400 kHz
void config_gyro_accel_default();      		// Turn on the gyro and accel with default pre-selected configurations
void config_mag_default();					// Turn on the magnetometer with default pre-selected configurations

void get_gyro(signed short int *output);					// output[0] = x | output[1] = y | output[2] = z
void get_accel(signed short int *output);				// output[0] = x | output[1] = y | output[2] = z
void get_mag(signed short int *output);					// output[0] = x | output[1] = y | output[2] = z


#endif