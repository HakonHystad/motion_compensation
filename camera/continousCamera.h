

#ifndef _CONTINOUSCAMERA_H_
#define _CONTINOUSCAMERA_H_

#include <atomic>
#include <mutex>
#include <unistd.h>

#include "camera.h"


using namespace AVT::VmbAPI;


namespace HH
{


// 1. define observer that reacts on new frames
class FrameObserver : public IFrameObserver
{
public :
// In your contructor call the constructor of the base class
// and pass a camera object
  FrameObserver ( CameraPtr pCamera, std::mutex &mtx, std::atomic<unsigned long long> &timestamp, std::atomic<int> &currentCam, int camNr )
	: IFrameObserver ( pCamera ),
	  m_currentCam( currentCam ),
    m_cam( camNr ),
	  m_mtx(mtx),
	  m_timestamp(timestamp)
	{
// Put your initialization code here
	}
    
    void FrameReceived ( const FramePtr pFrame )
	{

	    VmbFrameStatusType eReceiveStatus;
	    if( VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus ) )
	    {
	      //	      std::cout << eReceiveStatus << std::endl;
	      if ( VmbFrameStatusComplete == eReceiveStatus )
		{
// Put your code here to react on a successfully received frame
		    std::lock_guard<std::mutex> guard(m_mtx);
		    std::cout << "Recieved camera "<< m_cam << std::endl;

		    m_currentCam = m_cam;
		    
		    VmbUint64_t time;
		    pFrame->GetTimestamp( time );
		    m_timestamp = time;

	

		}

	    }
// When you are finished copying the frame , re - queue it
	    m_pCamera -> QueueFrame ( pFrame );
	}
private:
    std::atomic<int> &m_currentCam;
    int m_cam;
    std::mutex &m_mtx;
    std::atomic<unsigned long long> &m_timestamp;
};
    

class ContinousCamera : public Camera
{

public:
    ContinousCamera( unsigned char *imageBuffer1, unsigned char *imageBuffer2, VmbInt64_t bufferSz, std::mutex &mtx, std::atomic<unsigned long long> &timestamp, std::atomic<int> &currentCam, const  std::string cameraID1, const std::string cameraID2 )
	: Camera( imageBuffer1, bufferSz, cameraID1, cameraID2 ),
	  m_frame1( new Frame( imageBuffer1, bufferSz )  ),
	  m_frame2( new Frame( imageBuffer2, bufferSz )  )
	{
	    // base constructor starts cameras

	  /*
	  VmbInt64_t nPLS;
	  FeaturePtr feature;
	  
	  m_cam2->GetFeatureByName("PayloadSize", feature );
	  feature->GetValue(nPLS);
	  std::cout << "Requires " << nPLS << " bytes, got " << bufferSz << std::endl;
	  */
	  
	    // announce frames
	  m_err( m_cam1->AnnounceFrame( m_frame1 ) );
	  m_err( m_cam2->AnnounceFrame( m_frame2 ) );

	    // tie observers to frames
	  m_frame1->RegisterObserver( IFrameObserverPtr( new FrameObserver ( m_cam1, mtx, timestamp, currentCam, 1 ) ) );
	  m_frame2->RegisterObserver( IFrameObserverPtr( new FrameObserver ( m_cam2, mtx, timestamp, currentCam, 2 ) ) );

	    // ready api for capture
	    m_err( m_cam1->StartCapture() );
	    m_err( m_cam2->StartCapture() );

	    m_err( m_cam1->GetFeatureByName( "AcquisitionStart", m_startAq1 ) );
	    m_err( m_cam1->GetFeatureByName( "AcquisitionStop", m_stopAq1 ) );
	    m_err( m_cam2->GetFeatureByName( "AcquisitionStart", m_startAq2 ) );
	    m_err( m_cam2->GetFeatureByName( "AcquisitionStop", m_stopAq2 ) );
	    
	}

    ~ContinousCamera()
	{
	    m_cam1->EndCapture();
	    m_cam1->FlushQueue();
	    m_cam1->RevokeAllFrames();

	    m_cam2->EndCapture();
	    m_cam2->FlushQueue();
	    m_cam2->RevokeAllFrames();

	}


    void startCapture( int camNr = 0)
	{
	  if( camNr==1 || camNr ==0 )
	    {
	      // cam1
	      m_err( m_cam1->QueueFrame(m_frame1) );
	      m_err( m_startAq1->RunCommand() );
	    }
	 
	  sleep(0.015);// try to get offset captures.. 
	  if( camNr==2 || camNr==0 )
	    {
	      // cam2
	      m_err( m_cam2->QueueFrame(m_frame2) );
	      m_err( m_startAq2->RunCommand() );
	    }
	}

    void stopCapture( int camNr = 0)
	{
	  if( camNr==1 || camNr ==0 )
	    m_stopAq1->RunCommand();
	  if( camNr==2 || camNr ==0 )
	    m_stopAq2->RunCommand();
	}



    

protected:

private:

    FramePtr m_frame1;
    FramePtr m_frame2;

    FeaturePtr m_startAq1;
    FeaturePtr m_stopAq1;
    FeaturePtr m_startAq2;
    FeaturePtr m_stopAq2;


};

}// namespace HH

#endif /* _CONTINOUSCAMERA_H_ */
