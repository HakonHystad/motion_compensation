
#define MAX_FPS 40

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <unistd.h>

#include "../filter/settings.cuh"
#include "../filter//utilities.cuh"
#include "../filter/filter.cuh"

#include "../robot/robot.h"
#include "../camera/camera.h"



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

    float initialStates[N_STATES] = {1.5, 0.92, 2.0, 0.0324, 0, 0,	\
				    3.1415, 0, 0, 0, 0, 0};
    
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

    HH::Camera *cam = NULL;
    try
    {
      //      cam = new HH::Camera( image, IM_H*IM_W, "02-2165A-07078", "02-2165A-07077" );
	cam = new HH::Camera( image, IM_H*IM_W, camID1, camID2 );
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

    std::cout << "starting movement\n";

    robot.pose[1] -= 2*initialStates[1];
    robot.move(true);


    while(!robot.poseReached())
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

	// TODO: image acquisition
	// image = something
    
	float *camera = d_camera1;// TODO: set which camera took the picture

	cudaMemcpyToArrayAsync(cuArray, 0, 0, image, IM_W*IM_H*sizeof(uchar) , cudaMemcpyHostToDevice,  memStream);
	checkCUDAError("mempcy texture");
	
	// TODO: get timestamp
	float prevTime = 0;
	float newTime = 1.0f/60;

	// save the measured pose
	robot.getPose( pose );
	for (int j = 0; j < 6; ++j)
	    fd_pose << pose[j] << " ";
	fd_pose << std::endl;

	
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


	for (int i = 0; i < 6; ++i)
	    fd_calc_pose << sir[i+X_IDX] << " ";
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

    delete cam;

    return 0;
}

