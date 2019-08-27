#include <stdint.h>
#include <math.h>
#include <string.h>
#include "xio.h"
#include "dsp.h"

static int device_id = 0; // Our unique device ID

int xio_startup( int id )
{
    i2c_init( 100000 );
    device_id = id; // Our unique 32-bit ID.
    return 192000; // Desired sample rate.
}

static byte MAX11603_read( int index )
{
    i2c_start(); i2c_write(0xCA+0); i2c_write(0x60+1+2*index); i2c_stop();
    i2c_start(); i2c_write(0xCA+1); timer_delay(100);
    byte value = i2c_read(); i2c_ack(1); i2c_stop();
    return value;
}

// Convert knob/parameter value to a linear or logarithmic value within a control range.

double scale_lin( double val, double min, double max ) { return min+val*(max-min); }
double scale_log( double val, double min, double max ) { return min*pow(10,val*log10(max/min)); }

// Non-real time control thread.  Used to sense pots/switches, calculate coefficients,
// and send updated parameter values/coefficients to the real-time DSP threads.

void xio_control( const int rcv_prop[6], int snd_prop[6], int dsp_prop[6] )
{
    double param_volume = MAX11603_read(0) / 100.0;
    double param_preamp = MAX11603_read(1) / 100.0;
    double param_chorus = MAX11603_read(2) / 100.0;
    double param_toneeq = MAX11603_read(3) / 100.0;

    static int state = 1; dsp_prop[0] = state;
    
    if( state == 1 ) // Master volume, preamp gain
    {
        dsp_prop[1] = FQ( param_volume );
        dsp_prop[2] = FQ( scale_lin( param_preamp, 0.200, 0.700 )); // Left gain
        dsp_prop[3] = FQ( scale_lin( param_preamp, 0.100, 0.400 )); // Right gain
    }
    else if( state == 2 ) // Preamp control
    {
        // Decrease low-end gain-stage response as gain is turned up.
        // DC blocker (1-pole high pass filter) coefficients for 10Hz to 50/100/200 Hz. 
        dsp_prop[1] = calc_1pole( scale_lin( param_preamp, 10, 50 )); // Amp stage 1
        dsp_prop[2] = calc_1pole( scale_lin( param_preamp, 10,100 )); // Amp stage 2
        dsp_prop[3] = calc_1pole( scale_lin( param_preamp, 10,200 )); // Amp stage 3
    }
    else if( state == 3 ) // Preamp control
    {
        // Increase gain-stage mid-range as gain is turned up.
        // Peaking filter: Frequency at 900 Hz, adjustable gain (0dB to 12dB), Q = 0.707.
        calc_peaking( dsp_prop+1, 800.0/192000, scale_lin(param_preamp,0,+12), 0.707 );
    }
    else if( state == 4 ) // Tone control
    {
        // Decrease high-mid response as gain is turned up to smooth out preamp distortion
        // 2nd order low-pass filter with adjustable gain and frequency (15kHz to 5kHz), Q = 0.707.
        calc_lowpass( dsp_prop+1, scale_log(param_preamp,20000,10000)/192000, 0.707 );
    }
    else if( state == 5 ) // Tone control
    {
        // Tone control knob to reduce high-frequency response
        calc_lowpass( dsp_prop+1, scale_log(param_toneeq,13000,3000)/192000, 0.707 );
    }
    else if( state == 6 ) // Tone/EQ, Semi-Parametric EQ: Lower Mids
    {
        // Increase low-middle response as gain is turned up to compensate for decreased preamp low-end gain.
        calc_lowshelf( dsp_prop+1, 300.0/192000, scale_lin(param_preamp,0,+12), 0.707 );
    }
    else if( state == 7 ) // Tone/EQ, Semi-Parametric EQ: Middle Frequencies
    {
        // Decrease middle response as gain is turned up to compensate for increased preamp mid emphasis.
        calc_peaking( dsp_prop+1, 1000.0/192000, scale_lin(param_preamp,0,-12), 0.707 );
    }
    else if( state == 8 ) // Tone/EQ, Semi-Parametric EQ: Upper Mids
    {
        // Increase high-mid response as gain is turned up to emphasis preamp presence.
        calc_highshelf( dsp_prop+1, 3000.0/192000, scale_lin(param_preamp,0,+12), 0.707 );
    }
    else if( state == 9 ) // Chorus delay time, depth, rate, blend
    {
        dsp_prop[1] = FQ( scale_lin( param_chorus, 0.03, 0.07 )); // Depth
        dsp_prop[2] = FQ( scale_lin( param_chorus, 1.3/192000, 2.5/192000 )); // Rate
        dsp_prop[3] = FQ( scale_lin( param_chorus, 0.0, 0.5 ));// Blend
    }
    if( ++state > 9 ) state = 1;
}

void xio_initialize( void ) {}

int preamp_upsample_cc[96] = // FIR 192k 8k 24k -120 L=4
{
    FQ(-0.00000013861),FQ(-0.00000395176),FQ(+0.00004880348),FQ(-0.00009472574),
    FQ(-0.00028257212),FQ(+0.00141057733),FQ(-0.00154574644),FQ(-0.00307777293),
    FQ(+0.01148487239),FQ(-0.01044092883),FQ(-0.01994352140),FQ(+0.10359038142),
    FQ(+0.15873191137),FQ(+0.02977729401),FQ(-0.02845652307),FQ(+0.00774282738),
    FQ(+0.00422954934),FQ(-0.00440658988),FQ(+0.00105878279),FQ(+0.00045488241),
    FQ(-0.00033593334),FQ(+0.00005008748),FQ(+0.00001064877),FQ(-0.00000220923),
    FQ(-0.00000081734),FQ(+0.00000000000),FQ(+0.00006126299),FQ(-0.00022006004),
    FQ(+0.00000000000),FQ(+0.00150236214),FQ(-0.00320526118),FQ(+0.00000000000),
    FQ(+0.01155494591),FQ(-0.02106486211),FQ(+0.00000000000),FQ(+0.13637245991),
    FQ(+0.13637245991),FQ(+0.00000000000),FQ(-0.02106486211),FQ(+0.01155494591),
    FQ(+0.00000000000),FQ(-0.00320526118),FQ(+0.00150236214),FQ(+0.00000000000),
    FQ(-0.00022006004),FQ(+0.00006126299),FQ(+0.00000000000),FQ(-0.00000081734),
    FQ(-0.00000220923),FQ(+0.00001064877),FQ(+0.00005008748),FQ(-0.00033593334),
    FQ(+0.00045488241),FQ(+0.00105878279),FQ(-0.00440658988),FQ(+0.00422954934),
    FQ(+0.00774282738),FQ(-0.02845652307),FQ(+0.02977729401),FQ(+0.15873191137),
    FQ(+0.10359038142),FQ(-0.01994352140),FQ(-0.01044092883),FQ(+0.01148487239),
    FQ(-0.00307777293),FQ(-0.00154574644),FQ(+0.00141057733),FQ(-0.00028257212),
    FQ(-0.00009472574),FQ(+0.00004880348),FQ(-0.00000395176),FQ(-0.00000013861),
    FQ(-0.00000380062),FQ(+0.00002837621),FQ(-0.00000000000),FQ(-0.00037960809),
    FQ(+0.00098532369),FQ(-0.00000000000),FQ(-0.00452074445),FQ(+0.00854642470),
    FQ(-0.00000000000),FQ(-0.02902839806),FQ(+0.06603911930),FQ(+0.16666654447),
    FQ(+0.06603911930),FQ(-0.02902839806),FQ(-0.00000000000),FQ(+0.00854642470),
    FQ(-0.00452074445),FQ(-0.00000000000),FQ(+0.00098532369),FQ(-0.00037960809),
    FQ(-0.00000000000),FQ(+0.00002837621),FQ(-0.00000380062),FQ(-0.00000013861)
};

int preamp_dnsample_cc[96] = // FIR 192k 16.1k 24k -64
{
    FQ(-0.00002259174),FQ(-0.00010484502),FQ(-0.00018383221),FQ(-0.00019491985),
    FQ(-0.00008348342),FQ(+0.00015137688),FQ(+0.00042954297),FQ(+0.00060355573),
    FQ(+0.00052066002),FQ(+0.00011308998),FQ(-0.00052777077),FQ(-0.00114305887),
    FQ(-0.00139129064),FQ(-0.00100874034),FQ(+0.00002059925),FQ(+0.00137644799),
    FQ(+0.00246757626),FQ(+0.00266034491),FQ(+0.00159473782),FQ(-0.00055427402),
    FQ(-0.00302018495),FQ(-0.00467479548),FQ(-0.00449888068),FQ(-0.00211479922),
    FQ(+0.00187062036),FQ(+0.00592696847),FQ(+0.00812664928),FQ(+0.00698357282),
    FQ(+0.00226650833),FQ(-0.00460871878),FQ(-0.01088385800),FQ(-0.01346111665),
    FQ(-0.01027899858),FQ(-0.00149996418),FQ(+0.01007181996),FQ(+0.01968488480),
    FQ(+0.02235633970),FQ(+0.01505536461),FQ(-0.00149404482),FQ(-0.02221444951),
    FQ(-0.03872863434),FQ(-0.04190411335),FQ(-0.02511018430),FQ(+0.01296243836),
    FQ(+0.06702269608),FQ(+0.12606342993),FQ(+0.17627517089),FQ(+0.20510315433),
    FQ(+0.20510315433),FQ(+0.17627517089),FQ(+0.12606342993),FQ(+0.06702269608),
    FQ(+0.01296243836),FQ(-0.02511018430),FQ(-0.04190411335),FQ(-0.03872863434),
    FQ(-0.02221444951),FQ(-0.00149404482),FQ(+0.01505536461),FQ(+0.02235633970),
    FQ(+0.01968488480),FQ(+0.01007181996),FQ(-0.00149996418),FQ(-0.01027899858),
    FQ(-0.01346111665),FQ(-0.01088385800),FQ(-0.00460871878),FQ(+0.00226650833),
    FQ(+0.00698357282),FQ(+0.00812664928),FQ(+0.00592696847),FQ(+0.00187062036),
    FQ(-0.00211479922),FQ(-0.00449888068),FQ(-0.00467479548),FQ(-0.00302018495),
    FQ(-0.00055427402),FQ(+0.00159473782),FQ(+0.00266034491),FQ(+0.00246757626),
    FQ(+0.00137644799),FQ(+0.00002059925),FQ(-0.00100874034),FQ(-0.00139129064),
    FQ(-0.00114305887),FQ(-0.00052777077),FQ(+0.00011308998),FQ(+0.00052066002),
    FQ(+0.00060355573),FQ(+0.00042954297),FQ(+0.00015137688),FQ(-0.00008348342),
    FQ(-0.00019491985),FQ(-0.00018383221),FQ(-0.00010484502),FQ(-0.00002259174)
};

// Volume, Gain, Bass, Middle, Treble, Depth, Rate, Blend

static void preamp_upsample_4x( int* samples, int* coeffs, int* state )
{
    samples[3] = 3 * dsp_fir24( samples[0], coeffs+ 0, state+ 0 );
    samples[2] = 3 * dsp_fir24( samples[0], coeffs+24, state+24 );
    samples[1] = 3 * dsp_fir24( samples[0], coeffs+48, state+48 );
    samples[0] = 3 * dsp_fir24( samples[0], coeffs+72, state+96 );
}

static void preamp_dnsample_4x( int* samples, int* coeffs, int* state )
{
    memmove( state+3, state, (96-4)*sizeof(int) );
    state[0] = samples[0]; state[1] = samples[1];
    state[2] = samples[2]; state[3] = samples[3];
    int ah = 0; unsigned al = 1<<(QQ-1);
    dsp_power24( &ah,&al, coeffs+ 0, state+ 0);
    dsp_power24( &ah,&al, coeffs+24, state+24);
    dsp_power24( &ah,&al, coeffs+48, state+48);
    dsp_power24( &ah,&al, coeffs+72, state+72);
    samples[0] = dsp_extract( ah, al );
}

static void preamp_emphasis_4x( int* samples, int* coeffs, int* state )
{
    samples[3] = dsp_biquad( samples[3], coeffs, state );
    samples[2] = dsp_biquad( samples[2], coeffs, state );
    samples[1] = dsp_biquad( samples[1], coeffs, state );
    samples[0] = dsp_biquad( samples[0], coeffs, state );
}

static void preamp_softclip_4x( int* samples, int gain ) // Amplify, clip, and invert waveform.
{
    if(  samples[3] < 0 ) samples[3] = +dsp_softclip( dsp_multiply( samples[3], -gain ));
    else                  samples[3] = -dsp_softclip( dsp_multiply( samples[3], +gain ));
    if(  samples[2] < 0 ) samples[2] = +dsp_softclip( dsp_multiply( samples[2], -gain ));
    else                  samples[2] = -dsp_softclip( dsp_multiply( samples[2], +gain ));
    if(  samples[1] < 0 ) samples[1] = +dsp_softclip( dsp_multiply( samples[1], -gain ));
    else                  samples[1] = -dsp_softclip( dsp_multiply( samples[1], +gain ));
    if(  samples[0] < 0 ) samples[0] = +dsp_softclip( dsp_multiply( samples[0], -gain ));
    else                  samples[0] = -dsp_softclip( dsp_multiply( samples[0], +gain ));
}

static void preamp_dc_block_4x( int* samples, int coeff, int* state )
{
    samples[3] = dsp_1pole_hp( samples[3], coeff, state );
    samples[2] = dsp_1pole_hp( samples[2], coeff, state );
    samples[1] = dsp_1pole_hp( samples[1], coeff, state );
    samples[0] = dsp_1pole_hp( samples[0], coeff, state );
}

int preampL_upsample_ss[96], preampL_dnsample_ss[96];
int preampR_upsample_ss[96], preampR_dnsample_ss[96];

int preampL1_state[2] = { 0,0 }, preampL2_state[2] = { 0,0 }, preampL3_state[2] = { 0,0 };
int preampR1_state[2] = { 0,0 }, preampR2_state[2] = { 0,0 }, preampR3_state[2] = { 0,0 };

int preamp_emph_cc[6] = { 0,0,0,0,0,0 };

int preampL1_emph_ss[4] = { 0,0,0,0 }, preampL2_emph_ss[4] = { 0,0,0,0 }, preampL3_emph_ss[4] = { 0,0,0,0 };
int preampR1_emph_ss[4] = { 0,0,0,0 }, preampR2_emph_ss[4] = { 0,0,0,0 }, preampR3_emph_ss[4] = { 0,0,0,0 };

int preamp_hicut_cc[6] = { 0,0,0,0,0,0 }, preamp_hicut_ss[4] = { 0,0,0,0 };

void xio_thread1( int samples[8], const int property[6] )
{
    int gain = 0; if( property[0] == 1 ) gain = property[2];
    samples[4] = samples[0];
    
    preamp_upsample_4x( samples+0, preamp_upsample_cc, preampL_upsample_ss );
    preamp_emphasis_4x( samples+0, preamp_emph_cc, preampL1_emph_ss );
    preamp_softclip_4x( samples+0, gain );
    preamp_dc_block_4x( samples+0, FQ(0.99990), preampL1_state );
    preamp_emphasis_4x( samples+0, preamp_emph_cc, preampL2_emph_ss );
    preamp_softclip_4x( samples+0, gain );
    preamp_dc_block_4x( samples+0, FQ(0.99950), preampL2_state );
    preamp_emphasis_4x( samples+0, preamp_emph_cc, preampL3_emph_ss );
    preamp_softclip_4x( samples+0, gain );
    preamp_dc_block_4x( samples+0, FQ(0.99900), preampL3_state );    
    preamp_dnsample_4x( samples+0, preamp_dnsample_cc, preampL_dnsample_ss );
}

void xio_thread2( int samples[8], const int property[6] )
{
    int gain = 0; if( property[0] == 1 ) gain = property[3];

    preamp_upsample_4x( samples+4, preamp_upsample_cc, preampR_upsample_ss );
    preamp_emphasis_4x( samples+4, preamp_emph_cc, preampR1_emph_ss );
    preamp_softclip_4x( samples+4, gain );
    preamp_dc_block_4x( samples+4, FQ(0.99995), preampR1_state );
    preamp_emphasis_4x( samples+4, preamp_emph_cc, preampR2_emph_ss );
    preamp_softclip_4x( samples+4, gain );
    preamp_dc_block_4x( samples+4, FQ(0.9999), preampR2_state );
    preamp_emphasis_4x( samples+4, preamp_emph_cc, preampR3_emph_ss );
    preamp_softclip_4x( samples+4, gain );
    preamp_dc_block_4x( samples+4, FQ(0.9995), preampR3_state );    
    preamp_dnsample_4x( samples+4, preamp_dnsample_cc, preampR_dnsample_ss );

    samples[1] = samples[4];
}

// Tone and master volume (Low-Pass with gain/attenuation).
    
int toneL_hcut_cc[6] = { 0,0,0,0,0,0 }, toneL_hcut_ss[4] = { 0,0,0,0 };
int toneL_eq01_cc[6] = { 0,0,0,0,0,0 }, toneL_eq01_ss[4] = { 0,0,0,0 };
int toneL_eq02_cc[6] = { 0,0,0,0,0,0 }, toneL_eq02_ss[4] = { 0,0,0,0 };
int toneL_eq03_cc[6] = { 0,0,0,0,0,0 }, toneL_eq03_ss[4] = { 0,0,0,0 };
int toneL_eq04_cc[6] = { 0,0,0,0,0,0 }, toneL_eq04_ss[4] = { 0,0,0,0 };

int toneR_hcut_cc[6] = { 0,0,0,0,0,0 }, toneR_hcut_ss[4] = { 0,0,0,0 };
int toneR_eq01_cc[6] = { 0,0,0,0,0,0 }, toneR_eq01_ss[4] = { 0,0,0,0 };
int toneR_eq02_cc[6] = { 0,0,0,0,0,0 }, toneR_eq02_ss[4] = { 0,0,0,0 };
int toneR_eq03_cc[6] = { 0,0,0,0,0,0 }, toneR_eq03_ss[4] = { 0,0,0,0 };
int toneR_eq04_cc[6] = { 0,0,0,0,0,0 }, toneR_eq04_ss[4] = { 0,0,0,0 };

void xio_thread3( int samples[8], const int property[6] )
{
    if( property[0] == 4 ) memcpy( toneL_hcut_cc, property+1, 5*sizeof(int) );
    if( property[0] == 5 ) memcpy( toneL_eq01_cc, property+1, 5*sizeof(int) );
    if( property[0] == 6 ) memcpy( toneL_eq02_cc, property+1, 5*sizeof(int) );
    if( property[0] == 7 ) memcpy( toneL_eq03_cc, property+1, 5*sizeof(int) );
    if( property[0] == 8 ) memcpy( toneL_eq04_cc, property+1, 5*sizeof(int) );
    
    samples[0] = dsp_biquad( samples[0], toneL_hcut_cc, toneL_hcut_ss );
    samples[0] = dsp_biquad( samples[0], toneL_eq01_cc, toneL_eq01_ss );
    samples[0] = dsp_biquad( samples[0], toneL_eq02_cc, toneL_eq02_ss );
    samples[0] = dsp_biquad( samples[0], toneL_eq03_cc, toneL_eq03_ss );
    samples[0] = dsp_biquad( samples[0], toneL_eq04_cc, toneL_eq04_ss );
    
    if( property[0] == 4 ) memcpy( toneR_hcut_cc, property+1, 5*sizeof(int) );
    if( property[0] == 5 ) memcpy( toneR_eq01_cc, property+1, 5*sizeof(int) );
    if( property[0] == 6 ) memcpy( toneR_eq02_cc, property+1, 5*sizeof(int) );
    if( property[0] == 7 ) memcpy( toneR_eq03_cc, property+1, 5*sizeof(int) );
    if( property[0] == 8 ) memcpy( toneR_eq04_cc, property+1, 5*sizeof(int) );
    
    samples[1] = dsp_biquad( samples[1], toneR_hcut_cc, toneR_hcut_ss );
    samples[1] = dsp_biquad( samples[1], toneR_eq01_cc, toneR_eq01_ss );
    samples[1] = dsp_biquad( samples[1], toneR_eq02_cc, toneR_eq02_ss );
    samples[1] = dsp_biquad( samples[1], toneR_eq03_cc, toneR_eq03_ss );
    samples[1] = dsp_biquad( samples[1], toneR_eq04_cc, toneR_eq04_ss );
}

// Multiple (three) voice modulated delay lines for flanger, chorus, delay

int delayL1_lfo_state[3] = {0,0,0},delayL2_lfo_state[3] = {0,0,0}, delayL3_lfo_state[3] = {0,0,0};
int delayR1_lfo_state[3] = {0,0,0},delayR2_lfo_state[3] = {0,0,0}, delayR3_lfo_state[3] = {0,0,0};

int delayL1_fifo_buffer[8192], delayL2_fifo_buffer[8192], delayL3_fifo_buffer[8192];
int delayR1_fifo_buffer[8192], delayR2_fifo_buffer[8192], delayR3_fifo_buffer[8192];

int delayL1_fifo_state[2] = {0,0}, delayL2_fifo_state[2] = {0,0}, delayL3_fifo_state[2] = {0,0};
int delayR1_fifo_state[2] = {0,0}, delayR2_fifo_state[2] = {0,0}, delayR3_fifo_state[2] = {0,0};

int delay_lfo( int rate, int* state )
{
    int angle = state[0], sin = state[1], cos = state[2];
    angle += rate; if( angle >= FQ(1.0) ) angle -= FQ(1.0);
    sin += dsp_multiply(rate,cos); cos -= dsp_multiply(rate,sin);
    state[0] = angle; state[1] = sin; state[2] = cos;
    return sin;
}

int delay_buffer( int* buffer, int* state, int sample, int lfo, int delay, int depth, int regen )
{
    int index = state[0], feedback = state[1];
    buffer[--index & 8191] = dsp_multiply(sample,FQ(1.0)-regen/8) + dsp_multiply(feedback,regen);
    feedback = buffer[(((delay & 0x0FFFE000) >> 13) + index) & 8191];
    int mm = delay + dsp_multiply( depth, lfo/2 + FQ(0.5) );       
    int ii = ((mm & 0x0FFFE000) >> 13) + index, ff = (mm & 0x00001FFF ) << 15;
    int x1 = buffer[ii&8191], x2 = buffer[(ii+1)&8191], x3 = buffer[(ii+2)&8191];
    sample = dsp_lagrange( ff, x1, x2, x3 );
    state[0] = index; state[1] = feedback;
    return sample;
}

void xio_thread4( int samples[8], const int property[6] )
{
    static int count = 1, volume = 0, rate = 0, depth = 0, blend = 0;
    if( property[0] == 1 ) { count = property[1]; volume = property[2]; }
    if( property[0] == 9 ) { rate = property[1]; depth = property[2]; blend = property[3]; }
    
    int rate1 = dsp_multiply( rate, FQ(0.83) ), depth1 = dsp_multiply( depth, FQ(0.99) );
    int rate2 = dsp_multiply( rate, FQ(1.00) ), depth2 = dsp_multiply( depth, FQ(0.89) );
    int rate3 = dsp_multiply( rate, FQ(0.67) ), depth3 = dsp_multiply( depth, FQ(0.79) );
    
    int lfo1 = delay_lfo( rate1, delayL1_lfo_state );
    int lfo2 = delay_lfo( rate2, delayL3_lfo_state );
    int lfo3 = delay_lfo( rate3, delayL3_lfo_state );
    
    // Modulation #1: Chorus, ~30 msec delay, no feedback
    int delay1 = delay_buffer( delayL1_fifo_buffer, delayL1_fifo_state,
                               samples[0], lfo1, FQ(0.77), depth1, FQ(0.0) );
    
    // Modulation #2: Chorus, ~23 msec delay, 10% feedback
    int delay2 = delay_buffer( delayL2_fifo_buffer, delayL2_fifo_state,
                               samples[0], lfo2, FQ(0.63), depth2, FQ(0.1) );
    
    // Modulation #3: Chorus, ~4 msec delay, 50% feedback
    int delay3 = delay_buffer( delayL3_fifo_buffer, delayL3_fifo_state,
                               samples[0], lfo3, FQ(0.13), depth3, FQ(0.4) );
    
    samples[0] = dsp_blend( samples[0], delay1, dsp_multiply(blend,FQ(1.00)) ) / 3
               + dsp_blend( samples[0], delay2, dsp_multiply(blend,FQ(0.50)) ) / 3
               + dsp_blend( samples[0], delay3, dsp_multiply(blend,FQ(0.20)) ) / 3;
}

void xio_thread5( int samples[8], const int property[6] )
{
    static int count = 1, volume = 0, rate = 0, depth = 0, blend = 0;
    if( property[0] == 1 ) { count = property[1]; volume = property[2]; }
    if( property[0] == 9 ) { rate = property[1]; depth = property[2]; blend = property[3]; }
    
    int rate1 = dsp_multiply( rate, FQ(0.70) ), depth1 = dsp_multiply( depth, FQ(0.99) );
    int rate2 = dsp_multiply( rate, FQ(1.00) ), depth2 = dsp_multiply( depth, FQ(0.89) );
    int rate3 = dsp_multiply( rate, FQ(0.50) ), depth3 = dsp_multiply( depth, FQ(0.79) );
    
    int lfo1 = delay_lfo( rate1, delayR1_lfo_state );
    int lfo2 = delay_lfo( rate2, delayR3_lfo_state );
    int lfo3 = delay_lfo( rate3, delayR3_lfo_state );
    
    // Modulation #1: Chorus, ~30 msec delay, no feedback
    int delay1 = delay_buffer( delayR1_fifo_buffer, delayR1_fifo_state,
                               samples[1], lfo1, FQ(0.70), depth1, FQ(0.0) );
    
    // Modulation #2: Chorus, ~23 msec delay, 10% feedback
    int delay2 = delay_buffer( delayR2_fifo_buffer, delayR2_fifo_state,
                               samples[1], lfo2, FQ(0.53), depth2, FQ(0.1) );
    
    // Modulation #3: Chorus, ~4 msec delay, 50% feedback
    int delay3 = delay_buffer( delayR3_fifo_buffer, delayR3_fifo_state,
                               samples[1], lfo3, FQ(0.09), depth3, FQ(0.5) );
    
    samples[0] = dsp_blend( samples[1], delay1, dsp_multiply(blend,FQ(1.00)) ) / 3
               + dsp_blend( samples[1], delay2, dsp_multiply(blend,FQ(0.50)) ) / 3
               + dsp_blend( samples[1], delay3, dsp_multiply(blend,FQ(0.20)) ) / 3;
}
