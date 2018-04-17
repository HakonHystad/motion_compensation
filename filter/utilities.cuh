#ifndef _UTILITIES_H_
#define _UTILITIES_H_

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <sys/time.h>
#include <iostream>
#include <string>

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





#endif /* _UTILITIES_H_ */
