#ifndef _SPHERICALPENDULUM_H_
#define _SPHERICALPENDULUM_H_

#include "rsi.h"
#include <cmath>
#include <valarray>




namespace HH
{

class SphericalPendulum : public RSI
{

public:
    SphericalPendulum( std::vector<double> startPose, std::vector<double> q_home, std::vector<double> base, std::vector<double> tool, HH::Manip manipulator, std::string port )
	: RSI( startPose, q_home, base, tool,  manipulator, port )
	{}
    ~SphericalPendulum()
	{}

    // for setting offsets, NBNB: not thread safe yet
    double& operator[]( const unsigned char idx )
	{
	    return m_offset[idx];
	}

    

    
//////////////////////////////////////////////////////////////
// convert between euler conventions
/////////////////////////////////////////////////////////////

    void XYZ_to_ZYX( const double in[3], double out[3] )
	{
	    double s1 = sin( in[0] );
	    double c1 = cos( in[0] );

	    double s2 = sin( in[1] );
	    double c2 = cos( in[1] );

	    double s3 = sin( in[2] );
	    double c3 = cos( in[2] );

	    double &A = out[0];
	    double &B = out[1];
	    double &C = out[2];

	    A = atan2( c1*s3+c3*s1*s2, c2*c3 );
	    double sA = sin( A );
	    double cA = cos( A );
	    
	    B = atan2( c1*c3*s2-s1*s3, c2*c3*cA + ( c1*s3 + c3*s1*s2 )*sA );
	    C = atan2( s2*sA+c2*s1*cA, (c1*c3-s1*s2*s3)*cA + c2*s3*sA );
	}

	    


protected:

    void addOffset( double pose[6] )
	{
	    KDL::Frame T = KDL::Frame( KDL::Rotation::EulerZYX( pose[3],pose[4],pose[5]), Vector(pose[0],pose[1],pose[2] ) );

	    double tmp[3];
	    XYZ_to_ZYX( &m_offset[3], tmp );

	    T = T*KDL::Frame( KDL::Rotation::EulerZYX( tmp[0], tmp[1], tmp[2]), Vector(m_offset[0],m_offset[1],m_offset[2] ) );

	    pose[0] = T.p.x();
	    pose[1] = T.p.y();
	    pose[2] = T.p.z();

	    T.M.GetEulerZYX( pose[3], pose[4], pose[5] );

	    
	}

    
    virtual void update( double newPose[6], std::vector<double> &currentPose, double interval )
	{
	    const double stepSz = 1e-4;
	    
	    
	    //////////////////////////////////////////////////////////////
	    // if new pose was given recently, just resend is
	    if( interval > stepSz )
	    {

		//////////////////////////////////////////////////////////////
		// else integrate previous pose with the dynamic equations

		currentPose[6] += PARAMETER::THETA_ALPHA;
		currentPose[7] += PARAMETER::THETA_BETA;



		int n_steps = interval/stepSz;
		double x = 0;

		const int POSE_SZ = currentPose.size();
		
		std::valarray<double> Y( POSE_SZ );


		// initial values
		for (int i = 0; i < POSE_SZ; ++i)
		    Y[i] = currentPose[i];


		for (int i = 0; i < n_steps; ++i)
		{
		    step( stepSz, x, Y ); 
		}

		for (int i = 0; i < POSE_SZ; ++i)
		    currentPose[i] = Y[i];
		
		currentPose[6] -= PARAMETER::THETA_ALPHA;
		currentPose[7] -= PARAMETER::THETA_BETA;

	    }

	    
	    for(int i = 0; i < 3; ++i)
		newPose[i] = currentPose[i];

	    XYZ_to_ZYX( &currentPose[6], &newPose[3] );

	    addOffset( newPose );
	}


private:

    std::vector<double> m_offset = {0,0,0,0,0,0};

//////////////////////////////////////////////////////////////
// RK 4 step
/////////////////////////////////////////////////////////////
    // ndep= number of variables, dx= stepsize, x = fraction of interval, Y=input/output, F=derivative function
    void step( double dx, double &x, std::valarray<double> &Y )
	{
	    int ndep = Y.size();
	    std::valarray<double> dY1(ndep), dY2(ndep), dY3(ndep), dY4(ndep);

	    dY1 = F( x           , Y             ) * dx;
	    dY2 = F( x + 0.5 * dx, Y + 0.5 * dY1 ) * dx;
	    dY3 = F( x + 0.5 * dx, Y + 0.5 * dY2 ) * dx;
	    dY4 = F( x +       dx, Y       + dY3 ) * dx;
	    Y += ( dY1 + 2.0 * dY2 + 2.0 * dY3 + dY4 ) / 6.0;

	    x += dx;
	}

//////////////////////////////////////////////////////////////
// derivative function 
/////////////////////////////////////////////////////////////

        std::valarray<double> F( double x, std::valarray<double> Y )
	{
	    std::valarray<double> f( Y.size() );

#define LINEAR_PARAMETERS Y[0],Y[1],Y[2],Y[3],Y[4],Y[5]// (double x, double y, double z, double x_dot, double y_dot, double z_dot)
#define ANGULAR_PARAMETERS Y[6],Y[7],Y[8],Y[9],Y[10],Y[11]//(double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot)

	    f[0] = dynamicsX( LINEAR_PARAMETERS );
	    f[1] = dynamicsY( LINEAR_PARAMETERS );
	    f[2] = dynamicsY( LINEAR_PARAMETERS );

	    f[3] = dynamicsX_d( LINEAR_PARAMETERS );
	    f[4] = dynamicsY_d( LINEAR_PARAMETERS );
	    f[5] = dynamicsY_d( LINEAR_PARAMETERS );

	    f[6] = dynamicsAlpha( ANGULAR_PARAMETERS );
	    f[7] = dynamicsBeta( ANGULAR_PARAMETERS );
	    f[8] = dynamicsGamma( ANGULAR_PARAMETERS );

	    f[9] = dynamicsAlpha_d( ANGULAR_PARAMETERS );
	    f[10] = dynamicsBeta_d( ANGULAR_PARAMETERS );
	    f[11] = dynamicsGamma_d( ANGULAR_PARAMETERS );
	    
	    return f;
	}


    inline double dynamicsX( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return x_dot;
	}

    inline double dynamicsY( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return y_dot;
	}

    inline double dynamicsZ( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return z_dot;
	}

    inline double dynamicsX_d( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return 0;
	}

    inline double dynamicsY_d( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return 0;
	}

    inline double dynamicsZ_d( double x, double y, double z, double x_dot, double y_dot, double z_dot )
	{
	    return 0;
	}


    inline double dynamicsAlpha( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return alpha_dot;
	}

    inline double dynamicsBeta( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return beta_dot;
	}
    
    inline double dynamicsGamma( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return gamma_dot;
	}


    // NB: uses XYZ euler angles in radians
    inline double dynamicsAlpha_d( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    double sinBeta = sin(beta),  cosBeta = cos(beta);
	    return ( alpha_dot*beta_dot*sinBeta - (9.81/PARAMETER::LENGTH)*sin(alpha) )/cosBeta;
	}

    inline double dynamicsBeta_d( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    double sinBeta = sin(beta),  cosBeta = cos(beta);
	    return -( (9.81/PARAMETER::LENGTH)*cos(alpha) + alpha_dot*alpha_dot*cosBeta )*sinBeta;
	}

    inline double dynamicsGamma_d( double alpha, double beta, double gamma, double alpha_dot, double beta_dot, double gamma_dot )
	{
	    return 0;
	}

};
    
}// namespace




#endif /* _SPHERICALPENDULUM_H_ */
