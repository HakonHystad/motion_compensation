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


//////////////////////////////////////////////////////////////
// class definition/
/////////////////////////////////////////////////////////////


class Robot
{

public:
    Robot(){};
    ~Robot(){};

    void move( bool convert = false )
	{
	    // convert to kuka A,B,C
	    if( convert )
		XYZ_to_kuka( &pose[3] );
	    // NB: m -> mm

	    // TODO: send pose
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
    void XYZ_to_kuka( float euler[] );

    // yeah its public..
    float pose[6];

protected:

private:
    bool ended = false;

};

// Kuka uses ZYX angles for A,B,C
void Robot::XYZ_to_kuka( float euler[] )
{
    float s1 = sin( euler[0] );
    float c1 = cos( euler[0] );

    float s2 = sin( euler[1] );
    float c2 = cos( euler[1] );

    float s3 = sin( euler[2] );
    float c3 = cos( euler[2] );

    float &A = euler[0];
    float &B = euler[1];
    float &C = euler[2];

    A = atan2( c1*s3+c3*s1*s2, c2*c3 );
    float sA = sin( A );
    float cA = cos( A );
	    
    B = atan2( c1*c3*s2-s1*s3, c2*c3*cA + ( c1*s3 + c3*s1*s2 )*sA );
    C = atan2( s2*sA+c2*s1*cA, (c1*c3-s1*s2*s3)*cA + c2*s3*sA );
}


//////////////////////////////////////////////////////////////
// async usage of robot
/////////////////////////////////////////////////////////////
void robotThreadFnc(std::mutex &mtx, std::condition_variable &convar, Robot &robot)
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
