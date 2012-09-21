// BRICS_MM - Mobile Manipulation Library
// Copyright (c) 2009 GPS GmbH
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// Example program that shows how BRICS_MM may interface with ROS
//

#include "ROSWrapper.h"
#include "ompl/OmplWrapper.h"
#include "utils/Logger.h"

#include <brics_3d_ros/WorldModelQueryServer.h>
#include <worldModel/WorldModel.h>
#include <worldModel/sceneGraph/DotVisualizer.h>
#undef LOGGER_H_ //unfortunately header gaurads are not unique...
#include <core/Logger.h>

#ifdef ENABLE_OSG
	#undef OSGVISUALIZER_H_ //unfortunately header gaurads are not unique...
	#include <worldModel/sceneGraph/OSGVisualizer.h>
#endif

using namespace std;
using namespace BRICS_MM;

//OSGVisualizer* osgVisualizer = 0;


int main(int argc, char* argv[])
{

	BRICS_MM::Logger::setMinLoglevel(BRICS_MM::Logger::LOGDEBUG);
	BRICS_3D::Logger::setMinLoglevel(BRICS_3D::Logger::LOGDEBUG);

	ros::init(argc, argv, "brics_mm");

	ros::NodeHandle n("~");
	ROSWrapper roswrapper(n);

	BRICS_3D::WorldModel* wm = new BRICS_3D::WorldModel();
	roswrapper.setWorldModel(wm);

#ifdef ENABLE_OSG
	// Visualization tool for world model
	BRICS_3D::RSG::OSGVisualizer* wmObserver = new BRICS_3D::RSG::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization
#endif

	BRICS_3D::WorldModelQueryServer wmServer(n, wm);
	wmServer.setServiceNameSpace("/brics_mm/worldModel/");
	wmServer.initialize();

	BRICS_3D::RSG::DotVisualizer* dbgObserver = new BRICS_3D::RSG::DotVisualizer(&wm->scene);
	wm->scene.attachUpdateObserver(dbgObserver);


	
	//create planner components
	PathPlannerComponent pathPlanner;
	RobotComponent robot;
	ConfigurationSpaceComponent cspace;
	CartesianSpaceComponent cartesianSpace;
	CollisionCheckerComponent collisionChecker;

	roswrapper.init(&pathPlanner, &robot, &cspace, &cartesianSpace, &collisionChecker);
	roswrapper.registerServices();

	brics_mm_ros::BRICSSelectPlanner::Request req;
	brics_mm_ros::BRICSSelectPlanner::Response res;

	ROS_INFO("Ready to plan a path.");

	ros::MultiThreadedSpinner spinner(2); // Use 2 threads
	spinner.spin();


}
