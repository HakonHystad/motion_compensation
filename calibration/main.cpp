
#include <iostream>
#include <cstdlib>// malloc
#include <unistd.h>// sleep
#include <fstream>

#define IM_H 768
#define IM_W 1024
#define MAX_FPS 40

#include "../camera/camera.h"
#include "../robot/robot.h"

bool writeImage( std::string name, unsigned char *data );

int main(int argc, char *argv[])
{
    //////////////////////////////////////////////////////////////
    // parse calibration configuration
    /////////////////////////////////////////////////////////////

    int n_images = 0;
    
    std::ifstream fd( "./data/calibration_poses.txt" );

    if ( !fd.is_open() )
    {
	std::cerr << "Could not open calibration poses file\n";
	exit( EXIT_FAILURE );
    }

    fd >> n_images;

    //////////////////////////////////////////////////////////////
    // initiate camera 
    /////////////////////////////////////////////////////////////


    unsigned char *image = (unsigned char*) std::malloc( IM_H*IM_W );

    HH::Camera *cam = NULL;
    try
    {
	cam = new HH::Camera( image, IM_H*IM_W );
    }catch(VmbErrorType)
    {
	exit( EXIT_FAILURE );
    }

    //////////////////////////////////////////////////////////////
    // sync cameras
    /////////////////////////////////////////////////////////////

    // not strictly neccessary for calibration, but a test for further use
    if( !cam->startPTP() )
	exit( EXIT_FAILURE );

    //////////////////////////////////////////////////////////////
    // capture images 
    /////////////////////////////////////////////////////////////

    auto robot = Robot();

    int i = 0;
    for (i = 0; i < n_images; ++i)
    {
	//////////////////////////////////////////////////////////////
	// move robot 
	/////////////////////////////////////////////////////////////

	// get pose from file
	for (int j = 0; j < 6; ++j)
	    fd >> robot.pose[j];// TODO? error handling
	
	// send move command
	robot.move();
	
	// wait for it to finish
	sleep( 1 );// TODO: something about this

	//////////////////////////////////////////////////////////////
	// acquire images
	/////////////////////////////////////////////////////////////

	// take image with camera 1
	if( !cam->capture( 1 ) )
	    break;

	std::string imageName = "./data/im_" + std::to_string(i);
	std::cout << "Took " << imageName << " at " << cam->getTimestamp() << std::endl;
	
	// save it	
	if( !writeImage( imageName + "a.pgm", image ) )
	    break;

	// take image with camera 2
	if( !cam->capture( 2 ) )
	    break;

	std::cout << "Took " << imageName << " at " << cam->getTimestamp() << std::endl;
	// and save it
	if( !writeImage( imageName + "b.pgm", image ) )
	    break;
	
    }

    fd.close();

    if( i != n_images )
	std::cerr << "Did not capture all images\n";
    else
	std::cout << "Done, made " << n_images << " pairs of calibration images\n";


    //////////////////////////////////////////////////////////////
    // save serial numbers
    /////////////////////////////////////////////////////////////
    std::ofstream fd_ids("./data/cameraIDs.txt", std::ios::trunc );

    if( !fd_ids.is_open() )
    {
	std::cerr << "Could not write camera IDs\n";
    }
    else
    {
	fd_ids << cam->getSerial( 1 );
	fd_ids << std::endl;
	fd_ids << cam->getSerial( 2 );
    }
    fd_ids.close();


    


    if( cam != NULL )
    {
	cam->shutdown();
	delete cam;
    }

    
    return 0;
}

bool writeImage( std::string name, unsigned char *data )
{
    // save the image as a ascii file in the PGM format
    
    std::ofstream fd(name, std::ios::trunc );

    if( !fd.is_open() )
    {
	std::cerr << "Could not make image\n";
	return false;
    }

    //////////////////////////////////////////////////////////////
    // header
    /////////////////////////////////////////////////////////////

    // grayscale image
    fd << "P2\n";
    // size of image
    fd << IM_W << " " << IM_H << std::endl;
    // max intensity
    fd << 255 << std::endl;

    //////////////////////////////////////////////////////////////
    // intensities
    /////////////////////////////////////////////////////////////

    for (int row = 0; row < IM_H; ++row)
    {
	for (int col = 0; col < IM_W; ++col)
	{
	    fd << (int)data[ row*IM_W + col] << " ";
	}
	
	fd << std::endl;
    }
    bool res = fd.fail();
    fd.close();
    return !res;
}


    
