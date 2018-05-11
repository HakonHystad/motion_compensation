typedef unsigned char uchar;

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#define PARTICLE_BLK_SZ 512
#define ROUND_TO_BLK_SZ( n ) ( (n + PARTICLE_BLK_SZ/2)/PARTICLE_BLK_SZ )*PARTICLE_BLK_SZ

#define INITIAL_N_PARTICLES 150000// set this as a nominal value, it will be rounded to nearest multiple of block size
#define N_PARTICLES ROUND_TO_BLK_SZ( INITIAL_N_PARTICLES )
#define N_STATES 12
#define STEP_SZ 0.003f
#define N_WPOINTS 4
#define N_LINE_SAMPLES 32

#define PARTICLE_DBG 0

#if PARTICLE_DBG
#undef N_PARTICLES
#undef PARTICLE_BLK_SZ
#define N_PARTICLES 32
#define PARTICLE_BLK_SZ 32
#endif


/* image config */
#define IM_W 1024// width
#define IM_H 768// height

#define IM_LAMBDA 4// pixel offset along line normal


/* pre-estimated hanger parameters */
namespace PARAMETER
{
    const float LENGTH = 1.0f;// meter
    const float THETA_ALPHA = 0;// rad
    const float THETA_BETA = 0;// rad
}

//////////////////////////////////////////////////////////////
// macros
/////////////////////////////////////////////////////////////
/* name access of state variable */
#define X_IDX 0
#define Y_IDX 1
#define Z_IDX 2
#define X_D_IDX 3
#define Y_D_IDX 4
#define Z_D_IDX 5
#define ALPHA_IDX 6
#define BETA_IDX 7
#define GAMMA_IDX 8
#define ALPHA_D_IDX 9
#define BETA_D_IDX 10
#define GAMMA_D_IDX 11

#define STATE_X states[X_IDX]
#define STATE_Y states[Y_IDX]
#define STATE_Z states[Z_IDX]
#define STATE_X_D states[X_D_IDX]
#define STATE_Y_D states[Y_D_IDX]
#define STATE_Z_D states[Z_D_IDX]
#define STATE_ALPHA states[ALPHA_IDX]
#define STATE_BETA states[BETA_IDX]
#define STATE_GAMMA states[GAMMA_IDX]
#define STATE_ALPHA_D states[ALPHA_D_IDX]
#define STATE_BETA_D states[BETA_D_IDX]
#define STATE_GAMMA_D states[GAMMA_D_IDX]
 


// noise deviatiom
#define X_SIGMA 0.003f
#define Y_SIGMA 0.003f
#define Z_SIGMA 0.003f
#define X_D_SIGMA 0.003f
#define Y_D_SIGMA 0.003f
#define Z_D_SIGMA 0.003f
#define ALPHA_SIGMA 0.003f
#define BETA_SIGMA 0.003f
#define GAMMA_SIGMA 0.003f
#define ALPHA_D_SIGMA 0.003f
#define BETA_D_SIGMA 0.003f
#define GAMMA_D_SIGMA 0.003f



#endif /* _SETTINGS_H_ */
