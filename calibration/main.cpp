
#include <iostream>
#include <cstdlib>

#include "../camera/camera.h"

#define IM_H 768
#define IM_W 1024

int main(int argc, char *argv[])
{
    //////////////////////////////////////////////////////////////
    // parse calibration configuration
    /////////////////////////////////////////////////////////////

    /*
     *
     */
    
    int n_images = 0;

    unsigned char *image = (unsigned char*) std::malloc( IM_H*IM_W );

    HH::Camera *cam = NULL;
    try
    {
	cam = new HH::Camera( image, IM_H*IM_W );
    }catch(VmbErrorType)
    {
	delete cam;
	exit( EXIT_FAILURE );
    }

    /*
    unsigned char *image;
    
    HH::Camera cam( image, 0 );

    cam.capture( 1 );

    std::cout << "Taken at " << cam.getTimestamp() << std::endl;
    */

    if( cam != NULL )
	delete cam;
    
    return 0;
}
