XIO - Digital audio effects development kit
==================================

XIO provides a light framework for developing audio processing applications running on 3Degrees XIO USB/DSP modules.

XIO development Kit:  https://github.com/markseel/xio

Compiler and Linker:  xTIMEcomposer (current version for FlexFX is 14.3.3) is available from www.xmos.com  

Introduction
--------------------------------

Getting Started
--------------------------------

1) Download and install the free XMOS development tools from www.xmos.com.

2) Obtain the XIO dev-kit for building your own apps from “https://github.com/markseel/xio”.    
     Download the ZIP file or use GIT …
```
git clone https://github.com/flexfx/flexfx.github.io.git
```

3) Set environment variables for using XMOS command-line build tools …
```
Windows:         c:\Program Files (x86)\XMOS\xTIMEcomposer\Community_14.3.3\SetEnv.bat
OS X or Linux:   /Applications/XMOS_xTIMEcomposer_Community_14.3.3/SetEnv.command
```

4) Add your own source code to a new 'C' file (e.g. ‘example.c’).

5) Build the application …
```
xcc -O3 -report -lquadflash xio.xn xio.o dsp.o example.c -o example.xe
```

6) Burn your firmware application directly to FLASH using the XMOS JTAG (XTAG-2 or XTAG3)
```
xflash --boot-partition-size 1048576 --no-compression --factory example.xe --upgrade 1 example.xe
 — or —
```

7) If XIO firmware already exists on the target board then new firmware can be uploaded MIDI. Be sure to replace
the version number with the actual XMOS tools version you're currently using.  Also be sure to use the proper
name for the USB-Serial adapter.  Python must be installed.
```
xflash --no-compression --factory-version 14.3 --upgrade 1 example.xe -o example.bin

Windows:         python xio.py /dev/tty.usbserial-FTXLYU7C example.bin
OS X or Linux:   python xio.py COM3 example.bin
```

Programming Interface
-----------------------------------------

The 'xio.h' file defines the application interface for USB/I2S audio and MIDI applications.

```C
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
```

DSP Library
-----------------------------------------

You can write your own DSP functions or use these pre-esisting functions.
```C
#ifndef INCLUDED_DSP
#define INCLUDED_DSP

#include "xio.h"

// FQ converts Q28 fixed-point to floating point, QF converts floating-point to Q28

#define QQ 28
#define FQ(hh) (((hh)<0.0)?((int)((double)(1u<<QQ)*(hh)-0.5)):((int)(((double)(1u<<QQ)-1)*(hh)+0.5)))
#define QF(xx) (((int)(xx)<0)?((double)(int)(xx))/(1u<<QQ):((double)(xx))/((1u<<QQ)-1))

static double pi = 3.14159265359;

#define RR (1<<(QQ-1))

// XMOS specific assembly macros and inline functions.
//
// MAC performs 32x32 multiply and 64-bit accumulation, SAT saturates a 64-bit result, EXT converts
// a 64-bit result to a 32-bit value (extract 32 from 64), LD2/ST2 loads/stores two 32-values
// from/to 64-bit aligned 32-bit data arrays at address PP. All 32-bit fixed-point values are QQQ
// fixed-point formatted.
//
// AH (high) and AL (low) form the 64-bit signed accumulator
// XX, YY, and AA are 32-bit QQQ fixed point values
// PP is a 64-bit aligned pointer to two 32-bit QQQ values

#define DSP_LD00( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 0]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD01( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 1]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD02( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 2]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD03( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 3]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD04( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 4]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD05( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 5]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD06( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 6]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD07( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 7]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD08( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 8]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD09( pp, xx, yy )    asm volatile("ldd %0,%1,%2[ 9]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD10( pp, xx, yy )    asm volatile("ldd %0,%1,%2[10]":"=r"(xx),"=r"(yy):"r"(pp));
#define DSP_LD11( pp, xx, yy )    asm volatile("ldd %0,%1,%2[11]":"=r"(xx),"=r"(yy):"r"(pp));

#define DSP_ST00( pp, xx, yy )    asm volatile("std %0,%1,%2[ 0]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST01( pp, xx, yy )    asm volatile("std %0,%1,%2[ 1]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST02( pp, xx, yy )    asm volatile("std %0,%1,%2[ 2]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST03( pp, xx, yy )    asm volatile("std %0,%1,%2[ 3]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST04( pp, xx, yy )    asm volatile("std %0,%1,%2[ 4]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST05( pp, xx, yy )    asm volatile("std %0,%1,%2[ 5]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST06( pp, xx, yy )    asm volatile("std %0,%1,%2[ 6]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST07( pp, xx, yy )    asm volatile("std %0,%1,%2[ 7]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST08( pp, xx, yy )    asm volatile("std %0,%1,%2[ 8]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST09( pp, xx, yy )    asm volatile("std %0,%1,%2[ 9]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST10( pp, xx, yy )    asm volatile("std %0,%1,%2[10]"::"r"(xx), "r"(yy),"r"(pp));
#define DSP_ST11( pp, xx, yy )    asm volatile("std %0,%1,%2[11]"::"r"(xx), "r"(yy),"r"(pp));

#define DSP_MUL( ah, al, xx, yy ) asm volatile("maccs %0,%1,%2,%3":"=r"(ah),"=r"(al):"r"(xx),"r"(yy),"0"(0),"1"(0) );
#define DSP_MAC( ah, al, xx, yy ) asm volatile("maccs %0,%1,%2,%3":"=r"(ah),"=r"(al):"r"(xx),"r"(yy),"0"(ah),"1"(al) );
#define DSP_EXT( ah, al, xx, qq ) asm volatile("lextract %0,%1,%2,%3,32":"=r"(xx):"r"(ah),"r"(al),"r"(QQ+qq));
#define DSP_SAT( ah, al )         asm volatile("lsats %0,%1,%2":"=r"(ah),"=r"(al):"r"(QQ),"0"(ah),"1"(al));
#define DSP_DIV( qq,rr,ah,al,xx ) asm volatile("ldivu %0,%1,%2,%3,%4":"=r"(qq):"r"(rr),"r"(ah),"r"(al),"r"(xx));
#define DSP_CRC( xx, seed, poly ) asm volatile("crc32 %0,%2,%3":"=r"(xx):"0"(xx),"r"(seed),"r"(poly));

inline int dsp_mul( int xx, int yy ) // RR = XX * YY
{
    int ah = 0; unsigned al = 1<<(QQ-1);
    asm("maccs %0,%1,%2,%3":"=r"(ah),"=r"(al):"r"(xx),"r"(yy),"0"(ah),"1"(al) );
    asm("lextract %0,%1,%2,%3,32":"=r"(ah):"r"(ah),"r"(al),"r"(QQ));
    return ah;
}
inline int dsp_mac( int xx, int yy, int zh, unsigned zl ) // RR = XX * YY + ZZ
{
    int ah = 0; unsigned al = 0;
    asm("maccs %0,%1,%2,%3":"=r"(ah),"=r"(al):"r"(xx),"r"(yy),"0"(zh),"1"(zl) );
    asm("lextract %0,%1,%2,%3,32":"=r"(ah):"r"(ah),"r"(al),"r"(QQ));
    return ah;
}
inline int dsp_ext( int ah, int al ) // RR = AH:AL >> (64-QQ)
{
    asm volatile("lextract %0,%1,%2,%3,32":"=r"(ah):"r"(ah),"r"(al),"r"(QQ));
    return ah;
}

// Math and filter functions.
//
// XX, CC, SS, Yn, MM, and AA are 32-bit fixed point samples/data in QQQ format
// DD is the distance (0<=DD<1) between the first two points for interpolation
// KK is a time constant, QQQ format
// Yn are the data points to be interpolated
// CC is array of 32-bit filter coefficients - length is 'nn' for FIR, nn * 5 for IIR
// SS is array of 32-bit filter state - length is 'nn' for FIR, nn * 4 for IIR
// SS is array of 32-bit filter state - length is 3 for DCBLOCK, 2 for state-variable filter
// CC length is 3/5/7 and SS length is 2/4/6 fir IIR1/IIR2/IIR3 respectively
// AH (high) and AL (low) form the 64-bit signed accumulator

int  dsp_multiply( int  xx, int yy ); // RR = XX * YY
int  dsp_multacc ( int  xx, int yy, int zH, unsigned zL ); // RR = XX * YY + ZZ
int  dsp_extract ( int  ah, int al ); // RR = AH:AL >> (64-QQ)
int  dsp_crc     ( int  xx, int seed, int poly ); // Running CRC of XX, 
int  dsp_random  ( int  gg, int seed ); // Random number, gg = previous value
int  dsp_negexp  ( int  xx ); // Base-e exponent: yy = 1 - e^(-8 * xx), 0 <= xx <= 1.0
int  dsp_sin     ( int  xx ); // SINE approximation: yy ~= sin( 2 * pi * xx ), 0.0 <= xx <= 1.0
int  dsp_1pole_hp( int  xx, int cc, int* ss ); // One-pole high-pass DC blocking filter
int  dsp_1pole_lp( int  xx, int cc, int* ss ); // One-pole low-pass DC blocking filter
int  dsp_blend   ( int  dry, int wet, int blend );    // 0 (100% dry) <= MM <= 1 (100% wet)
int  dsp_interp  ( int  dd, int y1, int y2 );         // 1st order (linear) interpolation
int  dsp_lagrange( int  dd, int y1, int y2, int y3 ); // 2nd order (Lagrange) interpolation
void dsp_statevar( int* xx, const int* cc, int* ss ); // x[0]=in,xx[0:2]=lp/bp/hp out, c[0]=Fc,cc[1]=Q
int  dsp_biquad  ( int  xx, int* cc, int* ss ); // 2nd order IIR (BiQuad) filter - cc[5]=b0,b1,b2,a1,a2
int  dsp_convolve( int* ah, unsigned* al, int xx, int* cc, int* ss ); // 24-tap convolution
void dsp_power24 ( int* ah, unsigned* al, int* cc, int* ss ); // AH:AL = cc[n] * ss[n]; n = 0...23
int  dsp_fir24   ( int  xx, const int* cc, int* ss ); // FIR filter of 24 taps
int  dsp_softclip( int xx ); // 0.0 <= x <= 1.0, linear gain (x ~= 0) is 8

void mix_coeffs( int* cc_dst, int* cc_src, int nn, int rr ); // Reorder FIR coeffs for ASRC

// FIXME: dsp_statevar is not working properly.

 
// Biquad filter coefficient calculation functions (do not use these in real-time DSP threads).
//
// CC is an array of floating point filter coefficients
// FF, QQ, GG are the filter frequency, filter Q-factor, and filter gain
// For low/high pass filters set QQ to zero to create a single-pole 6db/octave filter
// GB/GM/GT are bass/mid/treble gains (0.0=min, 1.0=max).
// VB/VM/Vt are bass/mid/treble freq variation from standard (new_freq = standard_freq * variation).

int  calc_1pole    ( double ff ); // Returns 1-pole filter coefficient.
void calc_notch    ( int cc[5], double ff, double qq );
void calc_lowpass  ( int cc[5], double ff, double qq );
void calc_highpass ( int cc[5], double ff, double qq );
void calc_allpass  ( int cc[5], double ff, double qq );
void calc_bandpassQ( int cc[5], double ff, double qq );
void calc_bandpassF( int cc[5], double ff1, double ff2 );
void calc_peaking  ( int cc[5], double ff, double gg, double qq );
void calc_lowshelf ( int cc[5], double ff, double gg, double qq );
void calc_highshelf( int cc[5], double ff, double gg, double qq );

#endif // #ifndef INCLUDED_DSP
```


Minimal Application
-----------------------------------------

The application programmer only needs to add control and audip processing code to create a complete application.  The code below is a complete application.  Just add code to the 'xio\_startup', 'xio\_control', 'xio\_mixer', and the 'xio\_threadN' functions.

```C
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "xio.h"
#include "dsp.h"

const char* product_name_string   = "Example";           // The product name
const char* usb_audio_output_name = "Example Audio Out"; // USB audio output name
const char* usb_audio_input_name  = "Example Audio In";  // USB audio input name
const char* usb_midi_output_name  = "Example MIDI Out";  // USB MIDI output name
const char* usb_midi_input_name   = "Example MIDI In";   // USB MIDI input name

const int audio_sample_rate     = 192000; // Default sample rate at boot-up
const int audio_clock_mode      = 0; // 0=internal/master,1=external/master,2=slave
const int usb_output_chan_count = 2;     // 2 USB audio class 2.0 output channels
const int usb_input_chan_count  = 2;     // 2 USB audio class 2.0 input channels
const int i2s_channel_count     = 2;     // ADC/DAC channels per SDIN/SDOUT wire (2,4,or 8)

const int i2s_sync_word[8] = { 0xFFFFFFFF,0x00000000,0,0,0,0,0,0 }; // I2S WCLK values per slot

int  xio_startup( int id ) { return 512; } // 24.576 Mhz oscillator / 48000 kHz sample rate = 512

void xio_control( const int rcv_prop[6], int snd_prop[6], int dsp_prop[6] ) {}

void xio_initialize( void ) {}

void xio_thread1( int samples[8], const int property[6] ) {}
void xio_thread2( int samples[8], const int property[6] ) {}
void xio_thread3( int samples[8], const int property[6] ) {}
void xio_thread4( int samples[8], const int property[6] ) {}
void xio_thread5( int samples[8], const int property[6] ) {}
```

FlexFX Properties
----------------------------------

XIO applications can be controlled using property exchanges over MIDI.
A property is composed of a 16-bit command ID and five 32-bit data words.
ID's 0x0000 through 0x000F are reserved.

An example property is shown below:

```
Device ID   = 0x8033       (Must be non-zero)
Param 1     = 0x11223344
Param 2     = 0x55667788
Param 3     = 0x99aabbcc
Param 4     = 0x01234567
Param 5     = 0x89abcdef
```

Properties are transfered via over MIDI using MIDI SYSEX messages.
The XIO framework handles parsing and rendering of MIDI SYSEX encapulated XIO data therefore the user
application need not deal with MIDI SYSEX - the audio firmware only sees the 16-bit ID and five 32-word property values.

```
ID     DIRECTION            DESCRIPTION

0x01   USB Bidirectional    Identify the device, return the 32-bit unique device ID
0x02   USB Bidirectional    Begin firmware upgrade, echoed back to host
0x03   USB Bidirectional    Next 32 bytes of firmware image data, echoed
0x04   USB Host to Device   End firmware upgrade and reset (no USB property echo!)
0x05   USB Bidirectional    Begin bulk data upload to FLASH, echoed back to host
0x06   USB Bidirectional    Next 32 bytes of bulk data, echoed
0x07   USB Host to Device   End bulk data upload.
```

#### FlexFX ID = 0x01: Identify; return ID (3DEGFLEX) and versions

```
USB host ---> [ 0x01,  0, 0, 0, 0, 0 ]
USB host <--- [ 0x01, ID, Reserved, Reserved, Reserved, Reserved ]
```
The USB host can use this property to solicit information about that attached device and determine whether or not it is a FlexFX device.  The device will return the flexfx signature word, the serial number, and will echo property words #4 and #5.

#### FlexFX ID = 0x02: Begin firmware upgrade

```
USB host ---> [ 0x02, 0, 0, 0, 0, 0 ]
USB host <--- [ 0x02, 0, 0, 0, 0, 0 ]
```
Open the FLASH device and erase it to begin the firmware upgrade process.

#### FlexFX ID = 0x03: Continue firmware upgrade

```
USB host ---> [ 0x03, data1, data2, data3, data4, data5 ]
USB host <--- [ 0x03, data1, data2, data3, data4, data5 ]
```
Write the next 40 bytes of firmware data to FLASH.

#### FlexFX ID = 0x04: End firmware upgrade

```
USB host ---> [ 0x04, 0, 0, 0, 0, 0 ]
USB host <--- [ 0x04, 0, 0, 0, 0, 0 ]
```
Close the FLASH device and end the firmware upgrade process.
