/* wrapper for communication with kuka */

#ifndef _ROBOT_H_
#define _ROBOT_H_

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "KUKAVARPROXY_movement.h"

//////////////////////////////////////////////////////////////
// class definition/
/////////////////////////////////////////////////////////////
namespace HH
{

class Robot
{

public:
    Robot()
	: m_kukaPose( {0} )
	{
	    // empty
	}
    ~Robot(){};


    void move( bool convert = false )
	{
	    // safety in case move is called without setting all new poses
	    double tmp[6];
	    for (int i = 0; i < 6; ++i)
		tmp[i] = static_cast<double>( pose[i] );

	    if( convert )
	    {
		// convert to kuka A,B,C (ZYX)
		XYZ_to_kuka( &tmp[3] );
		// m -> mm
		for (int i = 0; i < 3; ++i)
		    tmp[i] *= 1000;
	    }


	    m_kukaPose.x = tmp[0];
	    m_kukaPose.y = tmp[1];
	    m_kukaPose.z = tmp[2];
	    m_kukaPose.a = tmp[3]*(180.0/3.1415926535);
	    m_kukaPose.b = tmp[4]*(180.0/3.1415926535);
	    m_kukaPose.c = tmp[5]*(180.0/3.1415926535);

	    std::cout << m_kukaPose.toString() << std::endl;

	    m_kuka.GoToPosition( m_kukaPose );
	    
	}

    void getPose( double currentPose[] )
	{
	    auto poseObj = m_kuka.CurrentPosition();
	    currentPose[0] = poseObj.x;
	    currentPose[1] = poseObj.y;
	    currentPose[2] = poseObj.z;
	    currentPose[3] = poseObj.a;
	    currentPose[4] = poseObj.b;
	    currentPose[5] = poseObj.c;

	}

    bool poseReached()
	{
	    return m_kuka.PositionReached( m_kukaPose );
	}
    
    void end()
	{
	    // TODO: release com
	    ended = true;
	}

    bool hasEnded()
	{
	    return ended;
	}




    // Kuka uses ZYX angles for A,B,C
    void XYZ_to_kuka( double euler[] );

    // yeah its public..
    float pose[6];

protected:

private:
    bool ended = false;

    KUKA::Kuka m_kuka;
    KUKA::RobotPose m_kukaPose;



};


// Kuka uses ZYX angles for A,B,C
void Robot::XYZ_to_kuka( double euler[] )
{
    double s1 = sin( euler[0] );
    double c1 = cos( euler[0] );

    double s2 = sin( euler[1] );
    double c2 = cos( euler[1] );

    double s3 = sin( euler[2] );
    double c3 = cos( euler[2] );

    double &A = euler[0];
    double &B = euler[1];
    double &C = euler[2];

    A = atan2( c1*s3+c3*s1*s2, c2*c3 );
    double sA = sin( A );
    double cA = cos( A );
	    
    B = atan2( c1*c3*s2-s1*s3, c2*c3*cA + ( c1*s3 + c3*s1*s2 )*sA );
    C = atan2( s2*sA+c2*s1*cA, (c1*c3-s1*s2*s3)*cA + c2*s3*sA );
}
}// namespace HH

//////////////////////////////////////////////////////////////
// async usage of robot
/////////////////////////////////////////////////////////////
void robotThreadFnc(std::mutex &mtx, std::condition_variable &convar, HH::Robot &robot)
{
    std::unique_lock<std::mutex> lock(mtx);
    while(true)
    {
	// release mutex and wait for signal to grab it
	convar.wait(lock);

	if( robot.hasEnded() )
	    break;
       	    
	//////////////////////////////////////////////////////////////
	// process new states 
	/////////////////////////////////////////////////////////////
	// TODO: prediction + smoothing?

	robot.move(true);

//	std::cout << robot.pose[0] << " " << robot.pose[3] << std::endl;
    

    }
    lock.unlock();
}


#endif /* _ROBOT_H_ */
