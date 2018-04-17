
//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "filter/settings.cuh"
#include "filter//utilities.cuh"
#include "filter/filter.cuh"


bool getConfig( int &nrOfImages, float camera1[], float camera2[], float states[], float worldPoints[] );
cudaTextureObject_t makeTexture( cudaArray *cuArray );


int main(int argc, char *argv[])
{
//////////////////////////////////////////////////////////////
// load test data
/////////////////////////////////////////////////////////////

    int nrOfImages;
    float camera1[12];
    float camera2[12];
    float states[N_STATES];
    float worldPoints[N_WPOINTS*3];

    if (!getConfig( nrOfImages, camera1, camera2, states, worldPoints) )
	exit( EXIT_FAILURE );

    float sigma[N_STATES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1,\
			     0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
/*

    float sigma[N_STATES] = {0.5, 0.5, 0.5, 0.1, 0.1, 0.1,\
			     0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    float offset[N_STATES] = { 0.5, 0.2, 0.3, 0.1, 0.1, 0.1};
    for (int i = 0; i < N_STATES; ++i)
    {
	states[i] += offset[i];
    }
*/  
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
    cudaMallocHost( (void**)&image, IM_W*IM_H*sizeof(uchar) );

    
    cudaArray *cuArray;
    // Allocate CUDA array in device memory
    auto channelDesc = cudaCreateChannelDesc<uchar>();
    cudaMallocArray(&cuArray, &channelDesc, IM_W, IM_H);
    checkCUDAError("malloc array");
    auto texObj = makeTexture( cuArray );


//////////////////////////////////////////////////////////////
// filter loop
/////////////////////////////////////////////////////////////

    std::ofstream fd("./data/results.txt", std::ios::trunc );

    if( !fd.is_open() )
    {
	std::cerr << "Could not open results.txt\n";
	exit( EXIT_FAILURE );
    }

    
    auto sir = Filter( states, sigma, worldPoints );

    #if PARTICLE_DBG
    nrOfImages = 2;// TODO: testing
    #endif

    float *camera = d_camera1;
    
    for (int i = 1; i <= nrOfImages; ++i)
    {
	std::string imageName = "./data/seq_" + std::to_string(i) + ".png";

	// load image onto device
	cv::Mat im = cv::imread( imageName, CV_LOAD_IMAGE_GRAYSCALE );
	image = im.data;
	cudaMemcpyToArray(cuArray, 0, 0, image, IM_W*IM_H*sizeof(uchar) , cudaMemcpyHostToDevice);
	checkCUDAError("mempcy texture");

	sir.update(0, 1.0f/60, camera, texObj );

	sir.resample();

	sir.mean();

	cudaThreadSynchronize();

	for (int i = 0; i < N_STATES; ++i)
	    fd << sir[i] << " ";
	fd << std::endl;

	if (camera==d_camera1)
	{
	    camera = d_camera2;
	}
	else
	{
	    camera = d_camera1;
	}


    }
       
    fd.close();
    return 0;
}




//////////////////////////////////////////////////////////////
// makeTexture
/////////////////////////////////////////////////////////////

cudaTextureObject_t makeTexture( cudaArray *cuArray )
{
    // Specify texture
    struct cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;
    resDesc.res.array.array = cuArray;

    // Specify texture object parameters
    struct cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
    texDesc.addressMode[0]   = cudaAddressModeBorder;// out of bounds = 0
    texDesc.addressMode[1]   = cudaAddressModeBorder;
    texDesc.filterMode       = cudaFilterModePoint;// nearest neighbor
    texDesc.readMode         = cudaReadModeElementType;
    texDesc.normalizedCoords = false;

    // Create texture object
    cudaTextureObject_t texObj = 0;
    cudaCreateTextureObject(&texObj, &resDesc, &texDesc, NULL);

    return texObj;
 
}


//////////////////////////////////////////////////////////////
// getConfig
/////////////////////////////////////////////////////////////

bool getConfig( int &nrOfImages, float camera1[], float camera2[], float states[], float worldPoints[] )
{
    std::ifstream fd( "./data/config.txt" );

    if ( !fd.is_open() )
    {
	std::cerr << "Could not open config file\n";
	return false;
    }

    // read nr of images
    fd >> nrOfImages;

    // read camera 1
    for (int i = 0; i < 12; ++i)
	fd >> camera1[i];

    // read camera 2
    for (int i = 0; i < 12; ++i)
	fd >> camera2[i];

    // read states
    for (int i = 0; i < N_STATES; ++i)
	fd >> states[i];
    
    // read world points
    for (int i = 0; i < N_WPOINTS*3; ++i)
	fd >> worldPoints[i];

    bool status = !fd.fail();

    fd.close();
    return status;

}
