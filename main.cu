
//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <iostream>
#include <string>

#include "filter/settings.cuh"
#include "filter//utilities.cuh"
#include "filter/filter.cuh"

#define _TEST_FILTER_

#ifdef _TEST_FILTER_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#endif



int main(int argc, char *argv[])
{
    //////////////////////////////////////////////////////////////
    // set initial values for the filter 
    /////////////////////////////////////////////////////////////
    
    float camera1[12];
    float camera2[12];
    float worldPoints[N_WPOINTS*3];

    // read config file
    if (!getConfig( camera1, camera2, worldPoints) )
	exit( EXIT_FAILURE );

    // x,y,z, x',y',z',a,b,g,a',b',g'
    float initialStates[N_STATES] = {1.6, -1.5, 2, 0.0324, 0, 0,\
				    0, 0, 0, 0, 0, 0};
    // 0.1m, 0.1m/s, ~5.7 deg, ~5.7deg/s
    float initialSigma[N_STATES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1,\
				    0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    /* GPU DATA */
    float *d_camera1;
    float *d_camera2;

    cudaMalloc( &d_camera1, sizeof(camera1) );
    cudaMalloc( &d_camera2, sizeof(camera2) );
    checkCUDAError("malloc main");

    cudaMemcpy( d_camera1, camera1, sizeof( camera1 ), cudaMemcpyHostToDevice );
    cudaMemcpy( d_camera2, camera2, sizeof( camera1 ), cudaMemcpyHostToDevice );
    checkCUDAError("memcpy main");

    //////////////////////////////////////////////////////////////
    // set up image loading
    /////////////////////////////////////////////////////////////
    uchar *image;
    #ifdef _TEST_FILTER_
    cv::Mat im = cv::imread( "./data/seq_500.png", CV_LOAD_IMAGE_GRAYSCALE );
    image = im.data;
    #endif
    /* end testing */
    cudaMallocHost( (void**)&image, IM_W*IM_H*sizeof(uchar) );

    
    cudaArray *cuArray;
    // Allocate CUDA array in device memory
    auto channelDesc = cudaCreateChannelDesc<uchar>();
    cudaMallocArray(&cuArray, &channelDesc, IM_W, IM_H);
    checkCUDAError("malloc array");
    auto texObj = makeTexture( cuArray );


    //////////////////////////////////////////////////////////////
    // make filter instance
    /////////////////////////////////////////////////////////////

    auto sir = Filter( initialStates, initialSigma, worldPoints );

    //////////////////////////////////////////////////////////////
    // get currect image from buffer
    /////////////////////////////////////////////////////////////

    // TODO: image acquisition
    // image = something
    
    float *camera = d_camera1;// TODO: set which camera took the picture
    
    // copy to GPU, TODO: asynchrounous
    
    #ifdef _TEST_FILTER_
    std::cout << "=======================================\n";
	
    std::cout << "Timings for " <<  N_PARTICLES << " particles\n";
    std::cout << "---------------------------------------\n";	

    double start, end, total=0;
    
    start = get_current_time();
    #endif
    
    cudaMemcpyToArray(cuArray, 0, 0, image, IM_W*IM_H*sizeof(uchar) , cudaMemcpyHostToDevice);
    checkCUDAError("mempcy texture");
	
    // TODO: get timestamp
    float prevTime = 0;
    float newTime = 1.0f/60;

    //////////////////////////////////////////////////////////////
    // perform filtering 
    /////////////////////////////////////////////////////////////

    
    sir.update(prevTime, newTime, camera, texObj );

    prevTime = newTime;

    sir.resample();

    sir.mean();

    cudaThreadSynchronize();

    #ifdef _TEST_FILTER_
    end = get_current_time();
    total += end-start;
    
    std::cout << "---------------------------------------\n";	
    std::cout << "Total time: " << total << "s\n";
    #endif

    //////////////////////////////////////////////////////////////
    // process new states 
    /////////////////////////////////////////////////////////////
    // TODO: prediction + smoothing?

    // TODO: send pose to robot as x,y,z in mm? and A,B,C in kuka euler coordinates


    // TODO: repeat with new image



    //////////////////////////////////////////////////////////////
    // clean up 
    /////////////////////////////////////////////////////////////


    cudaFree( d_camera1 );
    cudaFree( d_camera2 );
    cudaFreeArray( cuArray );
    cudaDestroyTextureObject(texObj);



    return 0;
}


