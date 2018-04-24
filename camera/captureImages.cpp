

// g++  --std=c++11 captureImages.cpp -o test.out -I/opt/Vimba_2_1/


//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////

#include <VimbaCPP/Include/VimbaCPP.h>// TODO: add to linker path


#include <iostream>
#include <string>
#include <unistd.h>
#include <fstream>

#define IM_W 1024
#define IM_H 768


    
using namespace AVT::VmbAPI;

//////////////////////////////////////////////////////////////
// main
/////////////////////////////////////////////////////////////
bool setDataRate( CameraPtr &camera, FeaturePtr &feature, int rate );
bool setGrayScale( CameraPtr &camera, FeaturePtr &feature );
bool enablePTP( CameraPtr &camera, FeaturePtr &feature, const char *mode );
bool writeImage( std::string name, unsigned char data[IM_H][IM_W] );

void soft_exit( VimbaSystem &sys )
{
    sys.Shutdown();
    exit( EXIT_FAILURE );
}



int main(int argc, char *argv[])
{
    //////////////////////////////////////////////////////////////
    // parse options
    /////////////////////////////////////////////////////////////

    int n_images = 0;
    if( argc == 2 )
    {
	n_images = std::stoi( argv[1] );
    }

    if( argc != 2 || n_images>35 || n_images < 1 )
    {
	std::cout << "Usage: give number of image pairs to acquire\n";
	return 1;
    }


    
    //////////////////////////////////////////////////////////////
    // prepare system
    /////////////////////////////////////////////////////////////

    VimbaSystem &system = VimbaSystem::GetInstance();

    if( system.Startup() != VmbErrorSuccess )
    {
	std::cerr << "Could not start Transport layer\n";
	soft_exit( system );
    }

    //////////////////////////////////////////////////////////////
    // look for cameras
    /////////////////////////////////////////////////////////////
	
    
    CameraPtrVector cameras;
    system.GetCameras( cameras );

    if( cameras.size() < 2 )
    {
	std::cerr << "Did not find 2 cameras\n";
	soft_exit( system );
    }


    // use serial number to link a static parameter to the calibrations
    std::string cameraID1;
    std::string cameraID2;
    cameras[0]->GetSerialNumber( cameraID1 );
    cameras[1]->GetSerialNumber( cameraID2 );

    std::cout << "Found two cameras:\n"
	      << cameraID1.c_str() << std::endl
	      << cameraID2.c_str() << std::endl;

    CameraPtr camera1;
    CameraPtr camera2;


    auto res1 = system.OpenCameraByID( cameraID1.c_str(), VmbAccessModeFull, camera1 );
    auto res2 = system.OpenCameraByID( cameraID2.c_str(), VmbAccessModeFull, camera2 );

    if( res1 != VmbErrorSuccess || res2 != VmbErrorSuccess )
    {
	std::cerr <<  "Could not open cameras\n";
	soft_exit( system );
    }

    //////////////////////////////////////////////////////////////
    // configure cameras
    /////////////////////////////////////////////////////////////

    FeaturePtr feature;

    // limit datarate to ensure the 2 camera setup works without loss
    /* To calculate the required minimum StreamBytesPerSecond setting for a
     * camera in any image mode, use the following formula:
     * StreamBytesPerSecond = Height x Width x FrameRate x Bytes per Pixel */
    int datarate = 768*1024*35*1;

    if( !setDataRate( camera1, feature, datarate ) )
	soft_exit( system );
    if( !setDataRate( camera2, feature, datarate ) )
	soft_exit( system );

    // take 8-bit grayscale images
    if( !setGrayScale( camera1, feature ) )
	soft_exit( system );
    if( !setGrayScale( camera2, feature ) )
	soft_exit( system );

    //////////////////////////////////////////////////////////////
    // sync cameras
    /////////////////////////////////////////////////////////////


    // for test purposes the PTP protocol is also used to sync camera clocks
    if( !enablePTP( camera1, feature, "master" ) )
	soft_exit( system );
    if( !enablePTP( camera2, feature, "slave" ) )
	soft_exit( system );

    std::cout << "Syncing cameras.. ";

    camera2->GetFeatureByName( "PtpStatus", feature );
    std::string status("");
	
    int wait = 10;// s
    int i;

    for (i = 0; i < 2*wait; ++i)
    {
	sleep(0.5);
	// poll status
	feature->GetDisplayName( status );

	if( status.compare("Slave") == 0 )
	    break;
    }
	
    if( i==2*wait )
    {
	std::cerr << "Could not sync\n";
	soft_exit( system );
    }

    std::cout << "Done\n";



    //////////////////////////////////////////////////////////////
    // acquire images
    /////////////////////////////////////////////////////////////

    
    FramePtr frame1;
    FramePtr frame2;

    for (i = 1; i <= n_images; ++i)
    {
	
	if( camera1->AcquireSingleImage(frame1, 5000) !=  VmbErrorSuccess )
	{
	    std::cerr << "Could not take image with camera 1\n";
	    soft_exit( system );
	}

	if( camera2->AcquireSingleImage(frame2, 5000) !=  VmbErrorSuccess )
	{
	    std::cerr << "Could not take image with camera 2\n";
	    soft_exit( system );
	}

    
	//////////////////////////////////////////////////////////////
	// save images to file
	/////////////////////////////////////////////////////////////

	std::string imageName = "./data/im_" + std::to_string(i);
	// save both images
	bool w_res = writeImage( imageName + "a.pgm", frame1.GetImage() );
	w_res = w_res && writeImage( imageName + "b.pgm", frame2.GetImage() );

	if( !w_res )
	{
	    std::cerr << "Unknown image write error\n";
	    soft_exit( system );
	}

	//////////////////////////////////////////////////////////////
	// check timestamps
	/////////////////////////////////////////////////////////////
	std::cout << "Took image 1 at: " << frame1.GetTimeStamp() << std::endl;
	std::cout << "Took image 2 at: " << frame2.GetTimeStamp() << std::endl;    

    }// for n_images

    
    //////////////////////////////////////////////////////////////
    // save serial numbers to file
    /////////////////////////////////////////////////////////////

    std::ofstream fd("camera_serial_numbers.txt", std::ios::trunc );

    if( !fd.is_open() )
    {
	std::cerr << "Could not save serial numbers\n";
	soft_exit( system );
    }
    // associate images of a with camera 1 serial nr
    fd << cameraID1 << std::endl;
    // and b with camera 2
    fd << cameraID2;

    fd.close();
    
    

    //////////////////////////////////////////////////////////////
    // clean up
    /////////////////////////////////////////////////////////////

    camera1->Close();
    camera2->Close();

    system.Shutdown();
	
    return 0;
}

bool setDataRate( CameraPtr &camera, FeaturePtr &feature, int rate )
{
    if( camera->GetFeatureByName( "StreamBytesPerSecond", feature ) == VmbErrorSuccess )
	feature->SetValue( rate );
    else
    {
	std::cerr << "Could not set datarate\n";
	return false;
    }
    return true;
}

bool setGrayScale( CameraPtr &camera, FeaturePtr &feature )
{
    if( camera->GetFeatureByName( "PixelFormat", feature ) == VmbErrorSuccess )
	feature->SetValue( VmbPixelFormatMono8 );
    else
    {
	std::cerr << "Could not set gray-scale\n";
	return false;
    }
    return true;
}

bool enablePTP( CameraPtr &camera, FeaturePtr &feature, const char *mode )
{
    if( camera->GetFeatureByName( "PtpMode", feature ) == VmbErrorSuccess )
	feature->SetValue( mode );
    else
    {
	std::cerr << "Could not enable PTP\n";
	return false;
    }
    return true;
}

bool writeImage( std::string name, unsigned char data[IM_H][IM_W] )
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
    // save intensities
    for (int row = 0; row < IM_H; ++row)
    {
	for (int col = 0; col < IM_W; ++col)
	{
	    fd << (int)data[row][col] << " ";
	}
	
	fd << std::endl;
    }
    bool res = fd.fail();
    fd.close();
    return !res;
}


    



