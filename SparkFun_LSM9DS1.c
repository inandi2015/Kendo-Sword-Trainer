//#include "NU32.h"               // constants, funcs for startup and UART
#include "SparkFun_LSM9DS1.h"
#include "LSM9DS1_Registers.h"
////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
//#include "config_1_3_2.h"
//// threading library
//
//// yup, the expander
//#include "port_expander_brl4.h"
//
//////////////////////////////////////
//// graphics libraries
//// SPI channel 1 connections to TFT
//#include "tft_master.h"
//#include "tft_gfx.h"
//#include "SparkFun_LSM9DS1.h"
//#include "LSM9DS1_Registers.h"
//// need for rand function
//#include <stdlib.h>
//// need for sin function
//#include <math.h>
#include <plib.h>

int i2c_master_setup(void);
void i2c_master_start(void);                // send a START signal
void i2c_master_restart(void);              // send a RESTART signal
void i2c_master_send(unsigned char byte);   // send a byte (either an address or data)
unsigned char i2c_master_recv(void);        // receive a byte of data
void i2c_master_ack(int val);               // send an ACK (0) or NACK (1)
void i2c_master_stop(void);                 // send a stop


// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Master will use I2C1 SDA1 (D9) and SCL1 (D10)
// Connect these through resistors to Vcc (3.3 V). 2.4k resistors recommended, 
// but something close will do.
// Connect SDA1 to the SDA pin on the slave and SCL1 to the SCL pin on a slave
//
int i2c_master_setup(void) {
  I2C1BRG = 90;                     // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // Fsck is the freq (100 kHz here), PGD = 104 ns
  I2C1CONbits.ON = 1;               // turn on the I2C1 module

  int status = 0, wait;             // Used to check is A, G and M is working
  char status_A_G, status_M;
  status_A_G = test_A_G(); 
  for(wait = 0; wait < 1000000; wait++){;}
  status_M = test_M();
  for(wait = 0; wait < 1000000; wait++){;}
  if(status_A_G == WHO_AM_I_AG_RSP && status_M == WHO_AM_I_M_RSP) // Ping both the gyro/accel and mag to check if it is working
    status = 1;
  return status;
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C1CONbits.SEN = 1;            // send the start bit
    while(I2C1CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C1CONbits.RSEN = 1;           // send a restart 
    while(I2C1CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C1TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C1STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C1STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
//    NU32_WriteUART1("I2C1 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C1CONbits.RCEN = 1;             // start receiving data
    while(!I2C1STATbits.RBF) { ; }    // wait to receive the data
    return I2C1RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C1CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C1CONbits.ACKEN = 1;            // send ACKDT
    while(I2C1CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C1CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C1CONbits.PEN) { ; }        // wait for STOP to complete
}

void start_comm(unsigned char sad, unsigned char ad){     // These steps are common for both write and read function
  i2c_master_start();
  i2c_master_send(sad << 1);
  i2c_master_send(ad);
}

int read_register(unsigned char sad, unsigned char ad, unsigned char * buffer, int n_bytes_read){ // Follows the sequence from tables 17 and 18 from the datasheet
  int read_index = 0;

  start_comm(sad, ad);
  i2c_master_restart();
  i2c_master_send((sad << 1) | 1) ;

  while(read_index < n_bytes_read){
    buffer[read_index] = i2c_master_recv();
    read_index++;

    if(read_index == n_bytes_read)
      i2c_master_ack(1);
    else
      i2c_master_ack(0);
  }
  i2c_master_stop();

  return 1;
}

int write_register(unsigned char sad, unsigned char ad, unsigned char * buffer, int n_bytes_write){ // Follows the sequence from tables 15 and 16 from the datasheet
  int write_index = 0;

  start_comm(sad, ad);

  // Send data
  while(write_index < n_bytes_write){
    i2c_master_send(buffer[write_index]);
    write_index++;
  }
  i2c_master_stop();

  return 1;
}

unsigned char test_A_G(){           // ping the WHO_A_I register to see if it's working, it should return 0x68 = 104
  unsigned char buffer[1];
  read_register(SAD_AG_1, WHO_AM_I_XG, buffer, 1);
  return buffer[0];
}

unsigned char test_M(){             // ping the WHO_A_I register to see if it's working, it should return 0x3D = 61
  unsigned char buffer[1];
  read_register(SAD_M_1, WHO_AM_I_M, buffer, 1);
  return buffer[0];
}

void config_gyro_accel_default(){      // Turn on the gyro and accel with default pre-selected configurations
  char buffer[1];
  int wait;
  buffer[0] = 0b01100001;        // See table 46 and 47 from the datasheet (Set Output Data Rate / Scale / DPS)
  write_register(SAD_AG_1, CTRL_REG1_G, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation

  buffer[0] = 0x00;              // See item 7.13 from the datasheet
  write_register(SAD_AG_1, CTRL_REG2_G, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation

  buffer[0] = 0x00;              // See table 51 from the datasheet (in this case low power is disable and ho HP filter is used)
  write_register(SAD_AG_1, CTRL_REG3_G, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation

  buffer[0] = 0b00111000;        // See table 62 from the datasheet (Enable all the axis and disable the interruption request)
  write_register(SAD_AG_1, CTRL_REG4, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation

  buffer[0] = 0x00;             // See table 54 from the datasheet (Define axis-orientation)
  write_register(SAD_AG_1, ORIENT_CFG_G, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation

  buffer[0] = 0b00111000;             // See table 64 from the datasheet (Define data decimation and enable axis)
  write_register(SAD_AG_1, CTRL_REG5_XL, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation

  buffer[0] = 0b11010000;             // See table 67 from the datasheet (Define power mode, scale, bandwidht and anti-aliasing)
  write_register(SAD_AG_1, CTRL_REG6_XL, buffer, 1);
  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation
}

void config_mag_default(){
  char buffer[1];
  int wait;
  buffer[0] = 0b10100001;
  write_register(SAD_AG_1, CTRL_REG1_G, buffer, 1);

  for(wait = 0; wait < 1000000; wait++){;}  // Wait before doing another operation
}

void get_sensor_data(char sad, char ad, signed short int *output){       // output[0] = x | output[1] = y | output[2] = z
  unsigned char buffer[6];                            // each value has 2 byter (3 values x 2 byter = 6 bytes)

  read_register(sad, ad, buffer, 6);                  // sequentially reads 6 registers

  output[0] = (buffer[1] << 8) | buffer[0];           // merge the 2 bytes in a single value, e.g. (OUT_X_H_G << 8) | OUT_X_L_G
  output[1] = (buffer[3] << 8) | buffer[2];           // xxx_x_H_x contains the most significant bytes and xxx_x_L_x contains the least significant bytes
  output[2] = (buffer[5] << 8) | buffer[4];           
}

void get_gyro(signed short int *output){                           // get data from gyro output[0] = x | output[1] = y | output[2] = z
  get_sensor_data(SAD_AG_1, OUT_X_L_G, output);
}

void get_accel(signed short int *output){                          // get data from accel output[0] = x | output[1] = y | output[2] = z 
  get_sensor_data(SAD_AG_1, OUT_X_L_XL, output);
}

void get_mag( signed short int *output){
  get_sensor_data(SAD_M_1, OUT_X_L_M, output);        // get data from mag output[0] = x | output[1] = y | output[2] = z
}
