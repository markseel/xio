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
