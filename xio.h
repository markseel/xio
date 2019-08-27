#ifndef INCLUDED_XIO_H
#define INCLUDED_XIO_H

typedef unsigned char byte;
typedef unsigned int  bool;

#include <stdint.h>

// The startup task is called before the control and mixer tasks and should be used to initizlize
// control data and variables. You must return the desired audio sample rate expressed as the
// system clock to sample rate ratio (Return = 24.576Mhz / desired_sample_rate).

extern int xio_startup( int id ); // Return MCLK/WCLK ratio (768 for 32kHz, 512 for 48kHz, etc).

// The control task is called at a rate of 1000 Hz and should be used for pot and switch sensing
// via I2C ADC's, handling of incoming properties from MIDI ('rcv_prop'), floating point based
// effects parameters computations, the generation of properties to be consumed by the DSP
// threads ('dsp_prop'), or generation of props to send to other devices via MIDI ('snd_prop').
// The incoming property 'rcv_prop' is only valid if its ID (rcv_prop[0])
// is non-zero. The out going property ID's (snd_prop[0], dsp_prop[0]) are only valid if their ID
// is non-zero. It's OK to use floating point calculations here and to do calculations that take
// a relatively long time since this thread is not a real-time audio thread.

extern void xio_control( const int rcv_prop[6], int snd_prop[6], int dsp_prop[6] );

// Audio Processing Threads. These functions are called once for each audio sample cycle. Do not
// use floating point operations since these functions are real-time audio threads - all DSP
// operations and calculations should be performed using fixed-point math.  Incoming audio ADC
// samples are stored in samples[0] (left channel) and samples[1] (right channel). Outgoing audio
// DAC samples should be stored in samples[0] (left channel) and samples[1] (right channel).
// Samples 2 through 7 are wrapped back around from thread5 to thread1.
// NOTE: IIR, FIR, and BiQuad coeff and state data *must* be declared non-static global!

// Process samples from the app_mixer function. Send results to stage 2.
extern void xio_thread1( int samples[8], const int property[6] );
// Process samples from stage 1. Send results to stage 3.
extern void xio_thread2( int samples[8], const int property[6] );
// Process samples from stage 2. Send results to stage 4.
extern void xio_thread3( int samples[8], const int property[6] );
// Process samples from stage 3. Send results to stage 5.
extern void xio_thread4( int samples[8], const int property[6] );
// Process samples from stage 4. Send results to the app_mixer function.
extern void xio_thread5( int samples[8], const int property[6] );

// On-board FLASH read write functions.

void flash_read ( int page, byte data[256] );
void flash_write( int page, const byte data[256] );

// Dynamic RAM read write functions.  Note that DRAM stores 16-bit values therfore the 32-bit
// 'data' parameter will be truncated to 16 bits.

void dram_write( int address, unsigned data );
int  dram_read ( int address );

// Functions for peripheral control (*** Only use these in the 'xio_control' function ***).
// I2C ans SPI share the same pins (SPI SCLK and I2C SCL, SPI MOSI and I2C SDA).
// For multiple SPI slave devices use an I2S bus expander to implement multiple SPI CSEL signals.
// Serial log output uses 1 start bit, 2 stop bits, and 1Mbaud rate.

void timer_delay( unsigned useconds );

void i2c_init ( int speed );  // Set bit rate (bps), set SCL/SDA to high
void i2c_start( void );       // Assert an I2C start condition.
void i2c_cont ( void );       // Continue with a restart.
bool i2c_write( byte value ); // Write 8-bit data value, returns ACK flag
byte i2c_read ( void );       // Read 8-bit data value.
void i2c_ack  ( bool ack );   // Assert the ACK/NACK bit after a read.
void i2c_stop ( void );       // Assert an I2C stop condition.

#endif // #ifndef INCLUDED_XIO_H
