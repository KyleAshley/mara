/**
 * \file Main.cpp
 * \brief Main entry point for Object Pose Estimation
 * \author Kester Duncan
 *
 * This file is the main entry point for using the object pose estimator using superquadrics by 
 * Kester Duncan
 */
#if 1
#include <iostream>
#include "SQTypes.h"
#include "ObjectPoseEstimator.h"


int main (int argc, char *argv[]) {
    ope::OPESettings settings;

    if (argc == 1)
    {
    	settings.fromFile = false;
    	settings.saveData = true;
   	}
   	else
   	{
   		settings.cloudFile = argv[1];
		  settings.fromFile = true;
		  settings.saveData = true;

   	}

    ope::ObjectPoseEstimator estimator(settings);
    ope::SQParameters sqParams = estimator.run();

    return 0;
}


#endif