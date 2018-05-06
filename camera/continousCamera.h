

#ifndef _CONTINOUSCAMERA_H_
#define _CONTINOUSCAMERA_H_

#include <atomic>
#include <mutex>


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
    FrameObserver ( CameraPtr pCamera, std::mutex &mtx, std::atomic<unsigned long long> &timestamp, unsigned char *im )
	: IFrameObserver ( pCamera ),
	  m_im( im ),
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
		if ( VmbFrameStatusComplete == eReceiveStatus )
		{
// Put your code here to react on a successfully received frame
		    std::lock_guard<std::mutex> guard(m_mtx);

		    if( pFrame->GetImage(m_im) != VmbErrorSuccess )
			return;

		    
		    VmbUint64_t time;
		    pFrame->GetTimestamp( time );
		    m_timestamp = time;
		    

		}

	    }
// When you are finished copying the frame , re - queue it
	    m_pCamera -> QueueFrame ( pFrame );
	}
private:
    unsigned char *m_im;
    std::mutex &m_mtx;
    std::atomic<unsigned long long> &m_timestamp;
};
    

class ContinousCamera : public Camera
{

public:
    ContinousCamera( unsigned char *imageBuffer1, unsigned char *imageBuffer2, VmbInt64_t bufferSz, std::mutex &mtx, std::atomic<unsigned long long> &timestamp, unsigned char *image, const  std::string cameraID1, const std::string cameraID2 )
	: Camera( imageBuffer1, bufferSz, cameraID1, cameraID2 ),
	  m_frame1( new Frame( imageBuffer1, bufferSz )  ),
	  m_frame2( new Frame( imageBuffer2, bufferSz )  )
	{
	    // base constructor starts cameras

	    // announce frames
	    m_cam1->AnnounceFrame( m_frame1 );
	    m_cam2->AnnounceFrame( m_frame2 );

	    // tie observers to frames
	    m_frame1->RegisterObserver( IFrameObserverPtr( new FrameObserver ( m_cam1, mtx, timestamp, image ) ) );
	    m_frame2->RegisterObserver( IFrameObserverPtr( new FrameObserver ( m_cam2, mtx, timestamp, image ) ) );

	    // ready api for capture
	    m_err( m_cam1->StartCapture() );
	    m_err( m_cam2->StartCapture() );

	    m_cam1->GetFeatureByName( "AcquisitionStart", m_startAq1 );
	    m_cam1->GetFeatureByName( "AcquisitionStop", m_stopAq1 );
	    m_cam2->GetFeatureByName( "AcquisitionStart", m_startAq2 );
	    m_cam2->GetFeatureByName( "AcquisitionStop", m_stopAq2 );
	    
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


    void startCapture()
	{
	    // cam1
	    m_cam1->QueueFrame(m_frame1);
	    m_startAq1->RunCommand();

	    // cam2
	    m_cam2->QueueFrame(m_frame2);
	    m_startAq2->RunCommand();
	    
	}

    void stopCapture()
	{
	    m_stopAq1->RunCommand();
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
