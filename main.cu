
#define MAX_FPS 40
#define _TEST_FILTER_

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <iostream>
#include <string>

#include "./filter/settings.cuh"
#include "./filter//utilities.cuh"
#include "./filter/filter.cuh"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;


int main(int argc, char *argv[])
{
    //////////////////////////////////////////////////////////////
    // set initial values for the filter 
    /////////////////////////////////////////////////////////////
    
    float camera1[12];
    float camera2[12];
    float worldPoints[N_WPOINTS*3];

    std::string camID1;
    std::string camID2;

    // read config file
    if (!getConfig( camera1, camera2, worldPoints, camID1, camID2) )
	exit( EXIT_FAILURE );

    float initialStates[N_STATES] = {1.5, 0, 2.0, 0, 0, 0,	\
				    0, 0, 0, 0, 0, 0};
    // 0.1m, 0.01m/s, ~5.7 deg, ~5.7deg/s
float initialSigma[N_STATES] = {0.1, 0.1, 0.1, 0.01, 0.01, 0.01,	\
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
    uchar *image1;
    uchar *image2;
    
    // make pinned memory
    cudaMallocHost( (void**)&image1, IM_W*IM_H*sizeof(uchar) );
    cudaMallocHost( (void**)&image2, IM_W*IM_H*sizeof(uchar) );

//    std::cout << "image 1: " << (void*)image1 << "\nimage 2: " << (void*)image2 << std::endl; 
    
    cudaArray *cuArray;
    // Allocate CUDA array in device memory
    auto channelDesc = cudaCreateChannelDesc<uchar>();
    cudaMallocArray(&cuArray, &channelDesc, IM_W, IM_H);
    checkCUDAError("malloc array");
    auto texObj = makeTexture( cuArray );

    // put image transfer and motion kernel in separate kernels to get async transfer
    cudaStream_t memStream = 0;
    cudaStream_t motionStream = 0;
    // comment out to run in default stream
    cudaStreamCreate(&memStream);
    cudaStreamCreate(&motionStream);
    checkCUDAError("make stream");




    //////////////////////////////////////////////////////////////
    // create instances
    /////////////////////////////////////////////////////////////

    auto sir = Filter( initialStates, initialSigma, worldPoints );

	
    //////////////////////////////////////////////////////////////
    // filter loop
    /////////////////////////////////////////////////////////////

    std::ofstream fd_calc_pose("./data/calculated_poses.txt", std::ios::trunc );
    if( !fd_calc_pose.is_open() )
    {
	std::cerr << "Could not open calc pose\n";
	exit( EXIT_FAILURE );
    }

    std::ofstream fd_timing("./data/timings.txt", std::ios::trunc );

    if( !fd_timing.is_open() )
    {
	std::cerr << "Could not open timings\n";
	exit( EXIT_FAILURE );
    }


    
    float *camera = d_camera1;
    int currentCam = 1;
    
    float prevTime = 0;
    float newTime = 1.0f/60;

    // left
//    cv::Mat im = cv::imread( "./calibration/left_segmented.png", CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat im = cv::imread( "./static_left_seg.png", CV_LOAD_IMAGE_GRAYSCALE );
    if( im.total() == IM_H*IM_W )
	memcpy(image1, im.data, im.total() );
    else
    {
	std::cerr << "Could not load image 1\n";
	exit(EXIT_FAILURE);
    }
    // right
    im = cv::imread( "./static_right_seg.png", CV_LOAD_IMAGE_GRAYSCALE );
//    cv::imread( "./calibration/right_segmented.png", CV_LOAD_IMAGE_GRAYSCALE );
    if( im.total() == IM_H*IM_W )
	memcpy(image2, im.data, im.total() );
    else
    {
	std::cerr << "Could not load image 2\n";
	exit(EXIT_FAILURE);
    }
    
    while(true)
    {
	
#ifdef _TEST_FILTER_
	std::cout << "=======================================\n";
	
	std::cout << "Timings for " <<  N_PARTICLES << " particles\n";
	std::cout << "---------------------------------------\n";	

	double start, end, total=0;
    
	start = get_current_time();
#endif

    
	//////////////////////////////////////////////////////////////
	// get currect image from buffer
	/////////////////////////////////////////////////////////////

	if( currentCam == 1 )
	  {
	    camera = d_camera1;
	    image = image1;
	    currentCam = 2;
	  }	  
	else if( currentCam == 2 )
	  {
	    camera = d_camera2;
	    image = image2;
	    currentCam = 1;
	  }
	else
	{
	    std::cerr << "Not expected currentCam\n";
	    break;
	}



	cudaMemcpyToArrayAsync(cuArray, 0, 0, image, IM_W*IM_H*sizeof(uchar) , cudaMemcpyHostToDevice,  memStream);
	checkCUDAError("mempcy texture");


	fd_timing << newTime - prevTime << std::endl;

	std::cout << "Integrated over " << newTime - prevTime << "s" << std::endl;

	//////////////////////////////////////////////////////////////
	// perform filtering 
	/////////////////////////////////////////////////////////////

    
	sir.update(prevTime, newTime, camera, texObj, motionStream );
	

	sir.resample();

	sir.mean();

	cudaThreadSynchronize();

	//////////////////////////////////////////////////////////////
	// pass on estimated states
	/////////////////////////////////////////////////////////////


	for (int i = 0; i < N_STATES; ++i)
	    fd_calc_pose << sir[i] << " ";
	fd_calc_pose << std::endl;



#ifdef _TEST_FILTER_
	end = get_current_time();
	total += end-start;
    
	std::cout << "---------------------------------------\n";	
	std::cout << "Total time: " << total << "s\n";
	std::cout << "States: ";
	for (int i = 0; i < N_STATES; ++i)
	    std::cout << sir[i] << " ";
	std::cout << std::endl;
#endif
       
    }



    //////////////////////////////////////////////////////////////
    // clean up 
    /////////////////////////////////////////////////////////////


    cudaFree( d_camera1 );
    cudaFree( d_camera2 );
    cudaFreeArray( cuArray );
    cudaDestroyTextureObject(texObj);
    cudaStreamDestroy(memStream);
    cudaStreamDestroy(motionStream);
    cudaFreeHost(image1);
    cudaFreeHost(image2);

    return 0;
}

