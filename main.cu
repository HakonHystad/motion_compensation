
//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "filter/settings.cuh"
#include "filter//utilities.cuh"
#include "filter/filter.cuh"

#include "robot/robot.h"

//#define _TEST_FILTER_

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
    #ifdef _TEST_FILTER_
    float initialStates[N_STATES] = {-0.5, 5.0, 1.0, 0.0324, 0.0, 0.0,\
				     0.261799, 0.261799, 0.0, 0.0, 0.0, 0.0};
    #else
    float initialStates[N_STATES] = {1.6, -1.5, 2, 0.0324, 0, 0,\
				    0, 0, 0, 0, 0, 0};
    #endif
    
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
    
    // make pinned memory
    cudaMallocHost( (void**)&image, IM_W*IM_H*sizeof(uchar) );

    
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
    auto robot = HH::Robot();


    // launch a separate thread for the robot processing
    std::mutex mtx;
    std::unique_lock<std::mutex> lock(mtx);
    lock.unlock();
    std::condition_variable convar;
    std::thread robotThread{robotThreadFnc, std::ref(mtx), std::ref(convar), std::ref(robot)};


    //////////////////////////////////////////////////////////////
    // filter loop
    /////////////////////////////////////////////////////////////
    // temp counter
    int k= 1;
    while(true)
    {
#ifdef _TEST_FILTER_
	std::string imageName = "./data/seq_" + std::to_string(k) + ".png";
	cv::Mat im = cv::imread( imageName, CV_LOAD_IMAGE_GRAYSCALE );
	memcpy(image, im.data, im.total() );
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

	// TODO: image acquisition
	// image = something
    
	float *camera = d_camera1;// TODO: set which camera took the picture

	cudaMemcpyToArrayAsync(cuArray, 0, 0, image, IM_W*IM_H*sizeof(uchar) , cudaMemcpyHostToDevice,  memStream);
	checkCUDAError("mempcy texture");
	
	// TODO: get timestamp
	float prevTime = 0;
	float newTime = 1.0f/60;


	// something is wrong, terminate threads nicely
	if(k++==3)// temp termination
	{
	    lock.lock();
	    robot.end();
	    lock.unlock();
	    convar.notify_all();
	    break;
	    // TODO: finish abort cases such as broken camera com etc.
        }
	
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


	// grab mutex
	lock.lock();
	
	for (int i = 0; i < 3; ++i)
	    robot.pose[i] = sir[i+X_IDX];
	for (int i = 0; i < 3; ++i)
	    robot.pose[i+3] = sir[i+ALPHA_IDX];

	// release mutex and let robot thread know to continue
	lock.unlock();
	convar.notify_all();


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

    robotThread.join();


    //////////////////////////////////////////////////////////////
    // clean up 
    /////////////////////////////////////////////////////////////


    cudaFree( d_camera1 );
    cudaFree( d_camera2 );
    cudaFreeArray( cuArray );
    cudaDestroyTextureObject(texObj);
    cudaStreamDestroy(memStream);
    cudaStreamDestroy(motionStream);
    cudaFreeHost(image);


    return 0;
}

