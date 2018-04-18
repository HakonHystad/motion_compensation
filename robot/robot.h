/* wrapper for communication with kuka */

#ifndef _ROBOT_H_
#define _ROBOT_H_

//////////////////////////////////////////////////////////////
// dependencies
/////////////////////////////////////////////////////////////
#include <cmath>


//////////////////////////////////////////////////////////////
// class definition/
/////////////////////////////////////////////////////////////


class Robot
{

public:
    Robot(){};
    ~Robot(){};

    void move( float pose[] )
	{
	    // convert to kuka A,B,C
	    XYZ_to_kuka( &pose[3] );
	    // NB: m -> mm

	    // TODO: send pose
	}


    // Kuka uses ZYX angles for A,B,C
    void XYZ_to_kuka( float euler[] );    

protected:

private:


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


#endif /* _ROBOT_H_ */
