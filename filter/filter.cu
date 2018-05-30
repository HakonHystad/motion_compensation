
#include "filter.cuh"


//////////////////////////////////////////////////////////////
// constructors
/////////////////////////////////////////////////////////////
Filter::Filter( const float mu[], const float sigma[],  const float worldPoints[] )
{
    
    // allocate memory on GPU
    cudaMalloc( &d_states1, N_PARTICLES*N_STATES*sizeof( float ) );
    cudaMalloc( &d_states2, N_PARTICLES*N_STATES*sizeof( float ) );
    cudaMalloc( &d_weights, (N_PARTICLES+1)*sizeof( float ) );
    cudaMalloc( &d_worldPoints, 3*N_WPOINTS*sizeof( float ) );
    cudaMalloc( &d_weightSum, sizeof( float ) );
    cudaMalloc( &d_resampleIdx, N_PARTICLES*sizeof( int ) );
    cudaMalloc( &d_meanStates, N_STATES*sizeof( float ) );
    cudaMalloc( &d_rngStates, N_PARTICLES*sizeof( curandState ) );
    cudaMalloc( &d_cumsum, (N_PARTICLES+1)*sizeof( float ) );
    checkCUDAError("constructor malloc");

    // copy world points to GPU
    cudaMemcpy( d_worldPoints, worldPoints, 3*N_WPOINTS*sizeof( float ), cudaMemcpyHostToDevice);
    checkCUDAError("constructor memcpy");

    // zero out first element of weights
    cudaMemset( d_weights, 0, sizeof( float ) );
    checkCUDAError("constructor memset");

    // Determine temporary device storage requirements for inclusive prefix sum
    d_temp_storage = NULL;
    temp_storage_bytes = 0;
    cub::DeviceScan::InclusiveSum( d_temp_storage, temp_storage_bytes, d_weights, d_cumsum, N_PARTICLES );
    // Allocate temporary storage for inclusive prefix sum
    cudaMalloc(&d_temp_storage, temp_storage_bytes);


    // set up the states of random generators on the GPU
    setupRNG_kernel<<<N_PARTICLES/PARTICLE_BLK_SZ, PARTICLE_BLK_SZ>>>( d_rngStates, 1234 );

    // initilize particles
    init( mu, sigma, worldPoints );
}

//////////////////////////////////////////////////////////////
// destructor
/////////////////////////////////////////////////////////////
Filter:: ~Filter()
{
    cudaFree(d_states1);
    cudaFree(d_states2);
    cudaFree(d_weights);
    cudaFree(d_worldPoints);
    cudaFree(d_weightSum);
    cudaFree(d_cumsum);
    cudaFree(d_meanStates);
    cudaFree(d_temp_storage);

}


//////////////////////////////////////////////////////////////
// initalize particles 
/////////////////////////////////////////////////////////////
void Filter::init( const float mu[], const float sigma[], const float worldPoints[] )
{
	    
    std::random_device rd{};
    std::mt19937 gen{rd()};

    float *h_states = new float[N_PARTICLES*N_STATES];
//  float h_states[N_PARTICLES*N_STATES];

    // pack memory for thread coalessing N_STATES*N_PARTICLES matrix
    for (int state = 0; state < N_STATES; ++state)
    {
	std::normal_distribution<float> distribution( mu[state], sigma[state] );

	for (int particle = 0; particle < N_PARTICLES; ++particle)
	{
	    h_states[ state*N_PARTICLES + particle ] = distribution( gen );
	}
    
    }

    

    
    // copy to GPU 
    cudaMemcpy( d_states1, h_states, N_PARTICLES*N_STATES*sizeof( float ), cudaMemcpyHostToDevice );
    checkCUDAError("memcpy");

    
    
    states = d_states1;
/* TODO: init on device
    // launch kernel
    dim3 blocksPerGrid( N_PARTICLES/PARTICLE_BLK_SZ, 1, 1 );
    dim3 threadsPerBlock( PARTICLE_BLK_SZ, 1, 1);

    init_kernel<<< blocksPerGrid, threadsPerBlock >>>(
*/

    delete[] h_states;
}


//////////////////////////////////////////////////////////////
// update motion and observation
/////////////////////////////////////////////////////////////
void Filter::update( float startTime, float endTime, const float *d_camera, cudaTextureObject_t texObj, cudaStream_t motionStream/*=0*/ )
{
    float interval = endTime - startTime;

    // launch kernel
    dim3 blocksPerGrid( N_PARTICLES/PARTICLE_BLK_SZ, 1, 1 );
    dim3 threadsPerBlock( PARTICLE_BLK_SZ, 1, 1);

    motion_kernel<<< blocksPerGrid, threadsPerBlock, 0, motionStream >>>( d_rngStates, interval, states );
    
//    cudaThreadSynchronize();

    
    dim3 blocksPerGrid2( N_PARTICLES, 1, 1 );
    dim3 threadsPerBlock2( N_LINE_SAMPLES*(N_WPOINTS/2), 1, 1);

    cudaMemset( d_weightSum, 0, sizeof(float) );

    observation_kernel<<< blocksPerGrid2, threadsPerBlock2 >>>( texObj, d_camera, d_worldPoints, states, &d_weights[1], d_weightSum );

    #if PARTICLE_DBG
    cudaThreadSynchronize();
    #endif
    checkCUDAError("update_kernel");
    
}

//////////////////////////////////////////////////////////////
// resample
/////////////////////////////////////////////////////////////
void Filter::resample()
{
    // normalize weights
    normalize();

    // launch kernel
    dim3 blocksPerGrid( N_PARTICLES/PARTICLE_BLK_SZ, 1, 1 );
    dim3 threadsPerBlock( PARTICLE_BLK_SZ, 1, 1);
 
    // swap elements
    if( states==d_states1 )
    {
	resample_kernel<<< blocksPerGrid, threadsPerBlock >>>( d_rngStates, d_cumsum, states, d_states2 );
	states = d_states2;
    }
    else
    {
	resample_kernel<<< blocksPerGrid, threadsPerBlock >>>( d_rngStates, d_cumsum, states, d_states1 );
	states = d_states1;
    }

//    cudaThreadSynchronize();
    checkCUDAError("resample_kernel");

}

void Filter::mean()
{
    // reset mean
    cudaMemset( d_meanStates, 0, N_STATES*sizeof(float) );
    checkCUDAError("memset mean");
    
    // launch kernel
    dim3 blocksPerGrid( N_PARTICLES/PARTICLE_BLK_SZ, 1, 1 );
    dim3 threadsPerBlock( PARTICLE_BLK_SZ, 1, 1);


    mean_kernel<<< blocksPerGrid, threadsPerBlock >>>( states, d_meanStates );
//    cudaThreadSynchronize();
    checkCUDAError("mean_kernel");

    // copy back mean states
    cudaMemcpy( h_meanStates, d_meanStates, N_STATES*sizeof( float ), cudaMemcpyDeviceToHost );
    checkCUDAError("memcpy mean");

    for (int i = 0; i < N_STATES; ++i)
    {
	h_meanStates[i] /= (float)N_PARTICLES;
    }

}


//////////////////////////////////////////////////////////////
// normalize and compute cumulative sum
/////////////////////////////////////////////////////////////
void Filter::normalize()
{
    // launch kernel
    dim3 blocksPerGrid( N_PARTICLES/PARTICLE_BLK_SZ, 1, 1 );
    dim3 threadsPerBlock( PARTICLE_BLK_SZ, 1, 1);

    cudaThreadSynchronize();
    normalize_kernel<<< blocksPerGrid, threadsPerBlock >>>( &d_weights[1], d_weightSum );

    checkCUDAError("normalize_kernel");

    /* compute cumulative sum of weights on the device */
    
    // Run inclusive prefix sum
    cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytes, d_weights, d_cumsum, N_PARTICLES );
    checkCUDAError("cumsum");
  
}


//////////////////////////////////////////////////////////////
// compute normal from 2 image points
/////////////////////////////////////////////////////////////
__device__ void computeNormal( const float *imPts, float *n )
{
    n[0] = imPts[1] - imPts[3];
    n[1] = imPts[2] - imPts[0];

    // make unit length
    float norm_inv = rhypotf( n[0], n[1] );
    n[0] *= norm_inv;
    n[1] *= norm_inv;
    
    // scale it by lambda pixels
    n[0] *= IM_LAMBDA;
    n[1] *= IM_LAMBDA;
}



/******************************* KERNELS ******************************************************/

//////////////////////////////////////////////////////////////
// mean kernel
/////////////////////////////////////////////////////////////

__global__ void mean_kernel( const float *statesPtr, float *meanStates )
{

    int index = blockDim.x*blockIdx.x + threadIdx.x;

    // block reduce to sum states
    typedef cub::BlockReduce<float, PARTICLE_BLK_SZ> BlockReduce;
    __shared__ typename BlockReduce::TempStorage tmp_storage;

    #pragma unroll
    for (int i = 0; i < N_STATES; ++i)
    {	
    
	float state = statesPtr[index + i*N_PARTICLES];
	    
	float blockSum = BlockReduce(tmp_storage).Sum( state );

	if( threadIdx.x==0 )
	    atomicAdd( &meanStates[ i ], blockSum );
    }

}

//////////////////////////////////////////////////////////////
// resample kernel
/////////////////////////////////////////////////////////////


__global__ void resample_kernel(  curandState *rngStates, const float *weights, const float *fromStates, float *toStates )
{
    int index = blockDim.x*blockIdx.x + threadIdx.x;
    
    curandState random = rngStates[index];

    float U = curand_uniform( &random );

    int particle = findInterval( U, weights );

    // copy states to new array
    #pragma unroll
    for (int i = 0; i < N_STATES; ++i)
    {
	toStates[ index + i*N_PARTICLES ] = fromStates[ particle + i*N_PARTICLES ];
    }
  
#if PARTICLE_DBG
    printf("Particle %d, sample: %f, resampled to particle %d\n", index, U,  particle);
#endif

    rngStates[index] = random;
}




//////////////////////////////////////////////////////////////
// normalize kernel
/////////////////////////////////////////////////////////////


__global__ void normalize_kernel( float *weightPtr, const float *weightSum )
{
    int index = blockDim.x*blockIdx.x + threadIdx.x;

    
    weightPtr[index] /= *weightSum;

    #if PARTICLE_DBG
    printf("Normalized %d: %f\n", index, weightPtr[index]);
    #endif
}



//////////////////////////////////////////////////////////////
// motion kernel
/////////////////////////////////////////////////////////////


__global__ void motion_kernel( curandState *rngStates, float time_left, float *statesPtr )
{
    int index = blockDim.x*blockIdx.x + threadIdx.x;

    // copy data to thread mem
    float states[N_STATES];

    for (int i = 0; i < N_STATES; ++i)
    {
	states[i] = statesPtr[ index + i*N_PARTICLES  ]; 
    }
#if PARTICLE_DBG
    printf("thread %d initial states: [ %f, %f, %f, %f, %f, %f ]\n", index, STATE_X, STATE_Y, STATE_Z, STATE_ALPHA, STATE_BETA, STATE_GAMMA ); 
#endif

    
    // convert to orientation to the center of mass
    STATE_ALPHA += PARAMETER::THETA_ALPHA;
    STATE_BETA += PARAMETER::THETA_BETA;

    // states: x,y,z,x_dot,y_dot,z_dot,alpha,beta,gamma,alpha_dot,beta_dot,gamma_dot
	    
    // first handle states with constant acceleration
	    
    STATE_X += time_left*STATE_X_D;// x
    STATE_Y += time_left*STATE_Y_D;// y
    STATE_Z += time_left*STATE_Z_D;// z
    // states[3] to states[5] and states[11] are unchanged 
    STATE_GAMMA += time_left*STATE_GAMMA_D;

    // next use the forward euler method to integrate alpha, beta and their velocities
    float alpha  = STATE_ALPHA;
    float beta = STATE_BETA;
    float alpha_d = STATE_ALPHA_D;
    
    while( time_left>STEP_SZ )
    {
		 
	STATE_ALPHA += STEP_SZ*alpha_d;// alpha
	STATE_BETA += STEP_SZ*STATE_BETA_D;// beta

	
	STATE_ALPHA_D += STEP_SZ*dynamicsAlpha( alpha, beta, alpha_d, STATE_BETA_D );// alpha_dot
	STATE_BETA_D += STEP_SZ*dynamicsBeta( alpha, beta, alpha_d );// beta_dot
 
	time_left -= STEP_SZ;

	alpha  = STATE_ALPHA;
	beta = STATE_BETA;
	alpha_d = STATE_ALPHA_D;
    }

    // finish up the remainder
    STATE_ALPHA += time_left*alpha_d;// - PARAMETER::THETA_ALPHA;// alpha
    STATE_BETA += time_left*STATE_BETA_D;// - PARAMETER::THETA_BETA;// beta

    STATE_ALPHA_D += time_left*dynamicsAlpha( alpha, beta, alpha_d, STATE_BETA_D );// alpha_dot
    STATE_BETA_D += time_left*dynamicsBeta( alpha, beta, alpha_d );// beta_dot

    // convert to pviot frame
    STATE_ALPHA -= PARAMETER::THETA_ALPHA;
    STATE_BETA -= PARAMETER::THETA_BETA;

    // apply noise
    curandState random = rngStates[index];

    STATE_X += curand_normal(&random)*X_SIGMA;
    STATE_Y += curand_normal(&random)*Y_SIGMA;
    STATE_Z += curand_normal(&random)*Z_SIGMA;
    STATE_X_D += curand_normal(&random)*X_D_SIGMA;
    STATE_Y_D += curand_normal(&random)*Y_D_SIGMA;
    STATE_Z_D += curand_normal(&random)*Z_D_SIGMA;
    STATE_ALPHA += curand_normal(&random)*ALPHA_SIGMA;
    STATE_BETA += curand_normal(&random)*BETA_SIGMA;
    STATE_GAMMA += curand_normal(&random)*GAMMA_SIGMA;
    STATE_ALPHA_D += curand_normal(&random)*ALPHA_D_SIGMA;
    STATE_BETA_D += curand_normal(&random)*BETA_D_SIGMA;
    STATE_GAMMA_D += curand_normal(&random)*GAMMA_D_SIGMA;

    rngStates[index] = random;
    // copy back to global memory
    for (int i = 0; i < N_STATES; ++i)
    {
	statesPtr[ index + i*N_PARTICLES  ] = states[i]; 
    }
    
#if PARTICLE_DBG
    printf("thread %d predicted states: [ %f, %f, %f, %f, %f, %f ]\n", index, STATE_X, STATE_Y, STATE_Z, STATE_ALPHA, STATE_BETA, STATE_GAMMA );
#endif

}

//////////////////////////////////////////////////////////////
// Observation kernel
/////////////////////////////////////////////////////////////

__global__ void observation_kernel(cudaTextureObject_t texObj, const float *cameraPtr, const float *worldPtsPtr, float *statesPtr, float *weightPtr, float *weightSum )
{

    __shared__ float camera[12];
    __shared__ float states[N_STATES];
    __shared__ float worldPts[3*N_WPOINTS];
    
    __shared__ float imPts[2*N_WPOINTS];
    __shared__ float normals[N_WPOINTS];

    // block reduce to sum weights
    typedef cub::BlockReduce<float, N_LINE_SAMPLES*(N_WPOINTS/2)> BlockReduce;
    __shared__ typename BlockReduce::TempStorage tmp_storage;


    // assuming 3*N_WPOINTS is largest
    if( threadIdx.x<3*N_WPOINTS )
    {
	worldPts[threadIdx.x] = worldPtsPtr[threadIdx.x];
	// next is states
#if N_STATES==12
	if( threadIdx.x < 12 )
	{
	    camera[threadIdx.x] = cameraPtr[threadIdx.x];
	    states[threadIdx.x] = statesPtr[ blockIdx.x + threadIdx.x*N_PARTICLES  ];
	}
#else
	if( threadIdx.x < 12 )
	    camera[threadIdx.x] = cameraPtr[threadIdx.x];

	if( threadIdx.x < N_STATES )
	    states[threadIdx.x] = statesPtr[ blockIdx.x + threadIdx.x*N_PARTICLES  ];
#endif

	// project to image points
	if( threadIdx.x < N_WPOINTS )
	{
	    float T[12];
	    statesToTransform( states, T );
	    float tmp[3];
	    // transform and project points
	    transformPt( T, &worldPts[3*threadIdx.x], tmp );
	    project( tmp, camera, &imPts[2*threadIdx.x] );

	    // and compute normals
	    if(  threadIdx.x < (N_WPOINTS/2) )
		    computeNormal( &imPts[4*threadIdx.x], &normals[2*threadIdx.x] );

	}	

    }

    __syncthreads();// is this neccesary if N_WPOINTS<32?

#if PARTICLE_DBG
    if(threadIdx.x==0)
	printf("Particle %d states: [ %f, %f, %f, %f, %f, %f ]\n", blockIdx.x, STATE_X, STATE_Y, STATE_Z, STATE_ALPHA, STATE_BETA, STATE_GAMMA ); 
#endif

 
    int idx = threadIdx.x/N_LINE_SAMPLES;
    int remainderIdx = threadIdx.x%N_LINE_SAMPLES;
    // fraction of line segment
    float frac  = (float)(remainderIdx+1)/( N_LINE_SAMPLES + 1 );

    
    // some aliases
    const float &ax = imPts[4*idx];
    const float &ay = imPts[4*idx+1];
    const float &bx = imPts[4*idx+2];
    const float &by = imPts[4*idx+3];

    const float *n = &normals[2*idx];

    // dont sample if line crosses image boundaries
    float weight = 0;
    float diff = 0;
    // negative pixel pos
    if( ax<IM_LAMBDA || ay<IM_LAMBDA || bx<IM_LAMBDA || by<IM_LAMBDA  )
	goto end;
    // too large pixel pos
    if( (IM_LAMBDA + ax)>=IM_W || (IM_LAMBDA + ay)>=IM_H || (IM_LAMBDA + bx)>=IM_W || (IM_LAMBDA + by)>=IM_H )
	goto end;

   
#if PARTICLE_DBG
    printf("particle %d, thread %d: a=[ %f, %f ], b=[ %f, %f ] \n", blockIdx.x, threadIdx.x,  ax, ay, bx, by );
#endif
	    

    float pt[2];

    pt[0] = ( 1-frac )*ax + frac*bx;
    pt[1] = ( 1-frac )*ay + frac*by;

#if PARTICLE_DBG
    printf("particle %d, thread %d: pt= [ %f, %f ]\n", blockIdx.x, threadIdx.x, pt[0], pt[1] );
#endif
    // measure image at point +- normal
    diff =  tex2D<uchar>( texObj, pt[0] + n[0], pt[1] + n[1]  ) - tex2D<uchar>( texObj, pt[0] - n[0], pt[1] - n[1] );

    weight = idx*diff*diff;// weight by position in array, closest to pivot first
       

#if PARTICLE_DBG
    printf("Particle %d, thread %d weight: %f \n", blockIdx.x, threadIdx.x, weight);
#endif

end:
    
    float blockSum = BlockReduce(tmp_storage).Sum( weight );


#if PARTICLE_DBG
    if(threadIdx.x==0)
	printf("Particle %d, blockSum: %f, weightSum: %f\n", blockIdx.x, blockSum, *weightSum );
#endif
    
    if(threadIdx.x==0)
    {
	weightPtr[blockIdx.x] = blockSum;
	atomicAdd( weightSum, blockSum );    
    }
}




/******************************* MOTION DEVICE FUNCTIONS **************************************/
//////////////////////////////////////////////////////////////
// motion equations
/////////////////////////////////////////////////////////////
__device__ inline float dynamicsAlpha( float alpha, float beta, float alpha_dot, float beta_dot )
{
    float sinBeta, cosBeta;
    sincosf( beta, &sinBeta, &cosBeta );
    return ( 2*alpha_dot*beta_dot*sinBeta - (9.81f/PARAMETER::LENGTH)*sinf(alpha) )/cosBeta;
}

__device__ inline float dynamicsBeta( float alpha, float beta, float alpha_dot )
{
    float sinBeta, cosBeta;
    sincosf( beta, &sinBeta, &cosBeta );
    return -( (9.81f/PARAMETER::LENGTH)*cosf(alpha) + alpha_dot*alpha_dot*cosBeta )*sinBeta;
}


/******************************* OBSERVATION DEVICE FUNCTIONS *********************************/

//////////////////////////////////////////////////////////////
// project world points to image points
/////////////////////////////////////////////////////////////
__device__ void projectWorldPt( const float *camera, const float *worldPt, const float states[], float imPt[]  )
{
    
	/* first update the camera to a new reference frame
	 * equivalent of transforming the states in the world
	 * frame to the camera frame*/
/*    
	float newCamera[12];

	transformCamera( camera, states, newCamera );

	// go through the world points and transform them to image points
	for (int i = 0; i < 2; ++i)
	{
	    project( &worldPts[i*3], newCamera, &imPts[i*2] );
	}
*/
    // transform point 
    float T[12];
	
    statesToTransform( states, T );

    float pt3D[3];
    transformPt( T, worldPt, pt3D );
    	    
    project( pt3D, camera, imPt );
	
}


//////////////////////////////////////////////////////////////
// transform a single world point
/////////////////////////////////////////////////////////////
__device__ void transformPt( const float T[12], const float *worldPt, float pt3D[3] )
{
    pt3D[0] = T[0]*worldPt[0] + T[1]*worldPt[1] + T[2]*worldPt[2] + T[3];
    pt3D[1] = T[4]*worldPt[0] + T[5]*worldPt[1] + T[6]*worldPt[2] + T[7];
    pt3D[2] = T[8]*worldPt[0] + T[9]*worldPt[1] + T[10]*worldPt[2] + T[11];
}





//////////////////////////////////////////////////////////////
//  Make a (truncated) transformation matrix given the states
/////////////////////////////////////////////////////////////
__device__ void statesToTransform( const float states[], float T[] )
{

    // Rotation matrix:
    float s1, c1, s2, c2, s3, c3;

    sincosf( STATE_ALPHA, &s1, &c1 );
    sincosf( STATE_BETA, &s2, &c2 );
    sincosf( STATE_GAMMA, &s3, &c3 );


//    printf("s1: %f, c1: %f\n", s1, c1);
    
    /* row 1 */
    // T_11
    T[0] = c2*c3;
    // T_12
    T[1] = -c2*s3;
    // T_13
    T[2] = s2;
    // T_14
    T[3] = STATE_X;
    /* row 2 */
    // T_21
    T[4] = c1*s3 + c3*s1*s2;
    // T_22
    T[5] = c1*c3 - s1*s2*s3;
    // T_23
    T[6] = -c2*s1;
    // T_24
    T[7] = STATE_Y;
    /* row 3 */
    // T_31
    T[8] = s1*s3 - c1*c3*s2;
    // T_32
    T[9] = c3*s1 + c1*s2*s3;
    // T_33
    T[10] = c1*c2;
    // T_34
    T[11] = STATE_Z;
    /* row 4 is only 0 0 0 1, save the memory */
/*
    #if PARTICLE_DBG
    printf("T= [ %f, %f, %f, %f;\n %f, %f, %f, %f;\n %f, %f, %f, %f ];\n", T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9], T[10], T[11] );
    #endif
*/

}



//////////////////////////////////////////////////////////////
// Transform camera to new reference frame given a state
/////////////////////////////////////////////////////////////
__device__ void transformCamera( const float *camera, const float states[], float newCamera[] )
{

    // first make transformation matrix from states
    float T[12];
    statesToTransform( states, T );
    
// update camera matrix by P*T
    for (int row = 0; row < 3; ++row)
    {
	float sum1 = 0;
	float sum2 = 0;
	float sum3 = 0;
	float sum4 = 0;
	
	for (int col = 0; col < 3; ++col)
	{
	    sum1 += camera[row*4 + col]*T[4*col];
	    sum2 += camera[row*4 + col]*T[4*col +1];
	    sum3 += camera[row*4 + col]*T[4*col +2];
	    sum4 += camera[row*4 + col]*T[4*col +3];
	}

	newCamera[row*4] = sum1;
	newCamera[row*4 +1] = sum2;
	newCamera[row*4 +2] = sum3;
	newCamera[row*4 +3] = sum4 + camera[row*4 +3];
    }
/*
    #if PARTICLE_DBG
    printf("camera= [ %f, %f, %f, %f;\n %f, %f, %f, %f;\n %f, %f, %f, %f ];\n", newCamera[0], newCamera[1], newCamera[2], newCamera[3], newCamera[4], newCamera[5], newCamera[6], newCamera[7], newCamera[8], newCamera[9], newCamera[10], newCamera[11] );
    #endif
*/
}


//////////////////////////////////////////////////////////////
// Project 3D world point using a given camera to a 2D point
/////////////////////////////////////////////////////////////
__device__ void project( const float worldPt[], const float *camera, float imPt[] )
{
    /* project [X,Y,Z,1]^T to [x,y,w] */
    
    // project x
    imPt[0] = worldPt[0]*camera[0] + worldPt[1]*camera[1] + worldPt[2]*camera[2] + camera[3];
    // project y
    imPt[1] = worldPt[0]*camera[4] + worldPt[1]*camera[5] + worldPt[2]*camera[6] + camera[7];
    // project w
    float denominator = worldPt[0]*camera[8] + worldPt[1]*camera[9] + worldPt[2]*camera[10] + camera[11];

    // normalize x and y, assume no division by 0
    imPt[0] /= denominator;
    imPt[1] /= denominator;
}

//////////////////////////////////////////////////////////////
// PRNG
/////////////////////////////////////////////////////////////
__global__ void setupRNG_kernel( curandState *rngStates, unsigned int seed )
{
    int index = blockDim.x*blockIdx.x + threadIdx.x;

    // same seed, different sequence, no offset
    curand_init( seed, index, 0, &rngStates[index] );
}



__device__ constexpr unsigned int floor_pow_2( unsigned int N )
{
#define SH_1( x ) (  x | (x >> 1) )
#define SH_2( x ) (  x | (x >> 2) )
#define SH_3( x ) (  x | (x >> 4) )
#define SH_4( x ) (  x | (x >> 8) )
#define SH_5( x ) (  x | (x >> 16) )
#define SH_6( x ) (  x - (x >> 1) )

#define FLOOR_POW_2( x ) ( SH_6( SH_5( SH_4( SH_3( SH_2( SH_1( x ) ) ) ) ) ) )
    return FLOOR_POW_2( N );

#undef SH_1
#undef SH_2
#undef SH_3
#undef SH_4
#undef SH_5
#undef SH_6
#undef FLOOR_POW_2
}

__device__ int findInterval( float val, const float *arr )
{
    int idx = 0;

    int n = floor_pow_2( N_PARTICLES );

    idx += ( arr[ n ] <= val )*( N_PARTICLES - n );

    // branchless binary search
    for (int i = n/2; i>1; i/=2)
	idx += (arr[ idx+i ]<= val)*i;

    
    // returns index of the last element in arr smaller than val
    return idx;
}
