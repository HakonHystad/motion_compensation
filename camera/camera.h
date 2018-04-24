#ifndef _CAMERA_HH_H_
#define _CAMERA_HH_H_

#include <VimbaCPP/Include/VimbaCPP.h>// TODO: add to linker path

#include <iostream>
#include <string>
#include <unistd.h>
#include <fstream>

#include "cameraError.h"

using namespace AVT::VmbAPI;



namespace HH
{

class Camera
{

public:

    // If ChunkModeActive = false Then PayloadSize = ImageSize
    Camera( unsigned char *imageBuffer, VmbInt64_t bufferSz, const  std::string cameraID1 = "", const std::string cameraID2 = "" ) :
	m_system( VimbaSystem::GetInstance() ),
	m_camID1( cameraID1 ),
	m_camID2( cameraID2 ),
	m_frame( new Frame( imageBuffer, bufferSz )  )
	{
	    m_err.setSystem( &m_system );
	    
	    // start system
	    m_err( m_system.Startup(), "Constructor" );


	    // if no cameras are specified do a search
	    if( m_camID1=="" )
		findCameras();

	    // open cameras for use
	    openCameras();

	    
	}
   
    ~Camera()
	{
//	    shutdown();
	}
    
    void findCameras()
	{
	    CameraPtrVector cameras;

	    std::cout << "Looking for 2 cameras\n";
    
	    m_err( m_system.GetCameras( cameras ), "findCameras" );

	    if( cameras.size() >= 2 )
	    {
		    cameras[0]->GetSerialNumber( m_camID1 );
		    cameras[1]->GetSerialNumber( m_camID2 );

		    std::cout << "Found two cameras:\n"
			      << m_camID1.c_str() << std::endl
			      << m_camID2.c_str() << std::endl;

	    }
	    else
		m_err( VmbErrorNotFound, "findCameras" );

	    
	}

    void openCameras()
	{
	    m_err( m_system.OpenCameraByID( m_camID1.c_str(), VmbAccessModeFull, m_cam1 ), "openCamera 1" );
	    m_err( m_system.OpenCameraByID( m_camID2.c_str(), VmbAccessModeFull, m_cam2 ), "openCamera 2" );	   
	}

    void configureCameras()
	{
	        FeaturePtr feature;

		// limit datarate to ensure the 2 camera setup works without loss
		/* To calculate the required minimum StreamBytesPerSecond setting for a
		 * camera in any image mode, use the following formula:
		 * StreamBytesPerSecond = Height x Width x FrameRate x Bytes per Pixel */
		int datarate = 768*1024*35*1;

		setDataRate( m_cam1, feature, datarate );
		setDataRate( m_cam2, feature, datarate );

		setGrayScale( m_cam1, feature );
		setGrayScale( m_cam2, feature );

	}

    bool startPTP( int timeout = 10 )
	{
	    FeaturePtr feature;
	    enablePTP( m_cam1, feature, "master" );
	    enablePTP( m_cam2, feature, "slave" );

	        std::cout << "Syncing cameras.. ";

		m_cam2->GetFeatureByName( "PtpStatus", feature );
		std::string status("");
	
		int i;

		for (i = 0; i < 2*timeout; ++i)
		{
		    sleep(0.5);
		    // poll status
		    feature->GetDisplayName( status );
		    std::cout << status << std::endl;// TODO remove once tested

		    if( status.compare("Slave") == 0 )
			break;
		}
	
		if( i==2*timeout )
		{
		    std::cerr << "Could not sync\n";
		    return false;
		}

		std::cout << "Done\n";
		return true;

	}

    bool capture( int cameraNr )
	{
	    VmbErrorType err;
	    if( cameraNr == 1 )
		err = m_cam1->AcquireSingleImage(m_frame, 5000);
	    else
		err = m_cam2->AcquireSingleImage(m_frame, 5000);

	    return ( err == VmbErrorSuccess );
	}

    int getTimestamp()
	{
	    VmbUint64_t time;
	    auto err = m_frame->GetTimestamp( time );

	    if( err != VmbErrorSuccess )
		return -1;
	    
	    return time;
	}



    void shutdown()
	{
	    std::cout << "shut down?\n";
	    
	    std::cout << m_system.Shutdown() << std::endl;
	    std::cout << "shut down\n";
	}



protected:

private:

    VimbaSystem &m_system;
    CameraError m_err;

    std::string m_camID1;
    std::string m_camID2;

    CameraPtr m_cam1;
    CameraPtr m_cam2;

    FramePtr m_frame;

    void setDataRate( CameraPtr &camera, FeaturePtr &feature, int rate )
	{
	    m_err( camera->GetFeatureByName( "StreamBytesPerSecond", feature ), "datarate" );
	    feature->SetValue( rate );	  
	}

    void setGrayScale( CameraPtr &camera, FeaturePtr &feature )
	{
	    m_err( camera->GetFeatureByName( "PixelFormat", feature ), "grayscale" );
	    feature->SetValue( VmbPixelFormatMono8 );
	}

    void enablePTP( CameraPtr &camera, FeaturePtr &feature, const char *mode )
	{
	    m_err( camera->GetFeatureByName( "PtpMode", feature ), "PTP" );
	    feature->SetValue( mode );
	}



};

}// namespace HH

#endif /* _CAMERA_HH_H_ */
