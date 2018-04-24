#ifndef _PTPOBSERVER_H_
#define _PTPOBSERVER_H_

#include <string>
#include <iostream>

#include "VimbaCPP/Include/IFeatureObserver.h"


/*

    // wait for ptp sync finish
    camera2->GetFeatureByName( "EventPtpSyncLocked", feature );
    
    feature->RegisterObserver( IFeatureObserverPtr( new PTPobserver() ) );


 */

using namespace AVT::VmbAPI;

class PTPobserver : public IFeatureObserver
{
    

public:
    void FeatureChanged( const FeaturePtr & feature )
	{
	    if ( feature != NULL )
	    {
		std::string strName ("");
		feature -> GetDisplayName ( strName );
		std::cout << "Event " << strName << " occurred " << std :: endl;

		
	    }
	}
};


#endif /* _PTPOBSERVER_H_ */
