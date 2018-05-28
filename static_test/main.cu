
#define MAX_FPS 40
#define _TEST_FILTER_
//#define _WITH_ROBOT_

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <unistd.h>
#include <mutex>

#include "../filter/settings.cuh"
#include "../filter//utilities.cuh"
#include "../filter/filter.cuh"

#include "../robot/robot.h"
#include "../camera/continousCamera.h"



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

#ifdef _WITH_ROBOT_
    float initialStates[N_STATES] = {1.5, 0.92, 2.0, 0.0324, 0, 0,	\
				    0, 0, 0, 0, 0, 0};
#else
    float initialStates[N_STATES] = {1.5, 0, 1.9, 0, 0, 0,	\
				    0, 0, 0, 0, 0, 0};
#endif
    // 0.1m, 0.01m/s, ~5.7 deg, ~5.7deg/s
    float initialSigma[N_STATES] = {0.1, 0.1, 0.1, 0.01, 0.01, 0.01,\
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

    HH::ContinousCamera *cam = NULL;
    unsigned long long prevTimestamp=0;
    std::atomic<unsigned long long> newTimestamp(0);
    std::atomic<int> currentCam(1);
    
    std::mutex mtx;
    std::unique_lock<std::mutex> lock(mtx);
    lock.unlock();
    
    
    try
    {
      //      cam = new HH::Camera( image, IM_H*IM_W, "02-2165A-07078", "02-2165A-07077" );
	cam = new HH::ContinousCamera( image1, image2, IM_H*IM_W, mtx, newTimestamp, currentCam, camID1, camID2 );
    }catch(VmbErrorType)
    {
	exit( EXIT_FAILURE );
    }
    // sync via precission time protocol
    if( !cam->startPTP() )
	exit( EXIT_FAILURE );



    //////////////////////////////////////////////////////////////
    // create instances
    /////////////////////////////////////////////////////////////

    auto sir = Filter( initialStates, initialSigma, worldPoints );

#ifdef _WITH_ROBOT_
    HH::Robot robot;

    for (int i = 0; i < 3; ++i)
	robot.pose[i] = initialStates[i];
    for (int i = 0; i < 3; ++i)
	robot.pose[i+3] = initialStates[i+ALPHA_IDX];

	std::cout << "Initiating at ";
	for( int i = 0; i<6; i++ )
	     std::cout << robot.pose[i] << " ";
	std::cout << std::endl;

    // move to start position
    robot.move(true);
    // wait for it to finish
    while( !robot.poseReached() )
	sleep( 0.2 );
#endif
	
    //////////////////////////////////////////////////////////////
    // filter loop
    /////////////////////////////////////////////////////////////
    double pose[6];
    std::ofstream fd_pose("./data/measured_poses.txt", std::ios::trunc );

    if( !fd_pose.is_open() )
    {
	std::cerr << "Could not open pose\n";
	exit( EXIT_FAILURE );
    }

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

    std::cout << "Starting capture" << std::endl;
    
    float *camera = d_camera1;
    
    cam->startCapture();// start continous capture
    float prevTime = 0;
    float newTime = 1.0f/60;

    std::cout << "starting movement\n";

#ifdef _WITH_ROBOT_
    robot.pose[1] -= 2*initialStates[1];
    robot.move(true);
#endif

    // use first image to set initial timestamp.. kind of a hack
    
    while( prevTimestamp >= newTimestamp )
    {
	// loop until new frame is in
	//std::cout << (float)newTimestamp/1e9 << std::endl;
      //std::cout << (newTimestamp - prevTimestamp)/1e9 << std::endl;
    }
    prevTimestamp = newTimestamp;
    prevTime = (float)prevTimestamp/1e9;


    //    cam->stopCapture();

#ifdef _WITH_ROBOT_
    while(!robot.poseReached())
    {
#else
	while(true)
	{
#endif
	
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
	/*
	if( currentCam == 1 )
	  cam->startCapture(2);
	else
	  cam->startCapture(1);
	*/

	while( (newTimestamp - prevTimestamp)<(0.005*1e9) )// require frames to be at least 0.5ms between eachother
	{
	    // loop until new frame is in
	    //std::cout << (float)newTimestamp/1e9 << std::endl;
//	  std::cout << (newTimestamp - prevTimestamp)/1e9 << std::endl;
	}
	
	lock.lock();

	if( currentCam == 1 )
	  {
	    camera = d_camera1;
	    image = image1;
	  }	  
	else if( currentCam == 2 )
	  {
	    camera = d_camera2;
	    image = image2;
	  }
	else
	{
	    std::cerr << "Not expected currentCam\n";
	    lock.unlock();
	    break;
	}

	
	// maybe should not be async b.c of mutex..
//	std::cout << "Address: " << (void*)image << std::endl;

//	cudaMemcpyToArrayAsync(cuArray, 0, 0, image, IM_W*IM_H*sizeof(uchar) , cudaMemcpyHostToDevice,  memStream);
	cudaMemcpyToArray(cuArray,0,0,image,IM_W*IM_H*sizeof(uchar), cudaMemcpyHostToDevice );
	checkCUDAError("mempcy texture");

	//cam->stopCapture(currentCam);
	prevTimestamp = newTimestamp;
	std::cout << "Using camera " << currentCam << ": " << (void*)image << std::endl;
	
	lock.unlock();
	
	newTime = (float)prevTimestamp/1e9;

	std::cout << "Integrated over " << newTime - prevTime << "s" << std::endl;
	// save time
	fd_timing << newTime - prevTime << std::endl;


	
#ifdef _WITH_ROBOT_
	// save the measured pose
	robot.getPose( pose );
	for (int j = 0; j < 6; ++j)
	    fd_pose << pose[j] << " ";
	fd_pose << std::endl;
#endif
	
	//////////////////////////////////////////////////////////////
	// perform filtering 
	/////////////////////////////////////////////////////////////

    
	sir.update(prevTime, newTime, camera, texObj, motionStream );
	prevTime = newTime;


	sir.resample();

	sir.mean();

	cudaThreadSynchronize();

	//////////////////////////////////////////////////////////////
	// pass on estimated states
	/////////////////////////////////////////////////////////////


	for (int i = 0; i < N_STATES; ++i)
	    fd_calc_pose << sir[i] << " ";
//	for (int i = 0; i < 3; ++i)
//	    fd_calc_pose << sir[i+ALPHA_IDX] << " ";
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

    cam->stopCapture();


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

    cam->shutdown();
    delete cam;

    return 0;
}

