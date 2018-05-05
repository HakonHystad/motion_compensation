#ifndef _UTILITIES_H_
#define _UTILITIES_H_

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <sys/time.h>
#include <iostream>
#include <string>
#include <fstream>

#include "settings.cuh"

//////////////////////////////////////////////////////////////
// timing
/////////////////////////////////////////////////////////////

double get_current_time()
{
   static int start = 0, startu = 0;
   struct timeval tval;
   double result;

   if (gettimeofday(&tval, NULL) == -1)
      result = -1.0;
   else if(!start) {
      start = tval.tv_sec;
      startu = tval.tv_usec;
      result = 0.0;
   }
   else
      result = (double) (tval.tv_sec - start) + 1.0e-6*(tval.tv_usec - startu);

   return result;
}


//////////////////////////////////////////////////////////////
// check cuda errors
/////////////////////////////////////////////////////////////
/* Utility function to check for and report CUDA errors */

void checkCUDAError(const std::string msg)
{
    cudaError_t err = cudaGetLastError();
    if( cudaSuccess != err) 
    {
	std::cerr << "Cuda error: " << msg << " " << cudaGetErrorString(err) << " \n";
        exit(EXIT_FAILURE);
    }                         
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

bool getConfig( float camera1[], float camera2[], float worldPoints[], std::string &camID1, std::string &camID2 )
{

    bool status = false;

    {
	std::ifstream fd( "./data/camera_parameters.txt" );

	if ( !fd.is_open() )
	{
	    std::cerr << "Could not open camera parameters file\n";
	    return false;
	}


	// read camera id 1
	fd >> camID1;
    
	// read camera 1
	for (int i = 0; i < 12; ++i)
	    fd >> camera1[i];

	// read camera id 1
	fd >> camID2;

	// read camera 2
	for (int i = 0; i < 12; ++i)
	    fd >> camera2[i];

	status = !fd.fail();
	fd.close();
    }

    {
	// read world points
	std::ifstream fd( "./data/config.txt" );

	if ( !fd.is_open() )
	{
	    std::cerr << "Could not open config file\n";
	    return false;
	}

    
	for (int i = 0; i < N_WPOINTS*3; ++i)
	    fd >> worldPoints[i];

	status &= !fd.fail();
	fd.close();

    }
    return status;
    
}






#endif /* _UTILITIES_H_ */
