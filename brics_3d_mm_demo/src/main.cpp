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
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/core/Logger.h>

#include <brics_3d_msgs/ResentSceneGraph.h>

#ifdef ENABLE_OSG
	#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#endif

using namespace std;


int main(int argc, char* argv[])
{

	brics_mm::Logger::setMinLoglevel(brics_mm::Logger::LOGDEBUG);
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	ros::init(argc, argv, "brics_mm");

	ros::NodeHandle n("~");
	brics_mm::ROSWrapper roswrapper(n);

	brics_3d::WorldModel* wm = new brics_3d::WorldModel();
	roswrapper.setWorldModel(wm);

#ifdef ENABLE_OSG
	// Visualization tool for world model
	brics_3d::rsg::OSGVisualizer* wmObserver = new brics_3d::rsg::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization
#endif

	brics_3d::WorldModelQueryServer wmServer(n, wm);
	wmServer.setServiceNameSpace("/brics_mm/worldModel/");
	wmServer.initialize();

	brics_3d::rsg::DotVisualizer* dbgObserver = new brics_3d::rsg::DotVisualizer(&wm->scene);
	wm->scene.attachUpdateObserver(dbgObserver);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	/* intialize data */
	std::stringstream serviceName;
	std::string serviceNameSpace = "/worldModel/";
	serviceName.str("");
	serviceName << serviceNameSpace << "resentSceneGraph";
	ros::ServiceClient resentSceneGraphClient = n.serviceClient<brics_3d_msgs::ResentSceneGraph>(serviceName.str());
	brics_3d_msgs::ResentSceneGraph resentSceneGraphRequest;
	resentSceneGraphRequest.request.subGraphRootId = wm->scene.getRootId();
	if (!resentSceneGraphClient.call(resentSceneGraphRequest)) {
		ROS_ERROR("Could not request to resent scene graph.");
	}
	
	//create planner components
	brics_mm::PathPlannerComponent pathPlanner;
	brics_mm::RobotComponent robot;
	brics_mm::ConfigurationSpaceComponent cspace;
	brics_mm::CartesianSpaceComponent cartesianSpace;
	brics_mm::CollisionCheckerComponent collisionChecker;

	roswrapper.init(&pathPlanner, &robot, &cspace, &cartesianSpace, &collisionChecker);
	roswrapper.registerServices();

	brics_mm_ros::BRICSSelectPlanner::Request req;
	brics_mm_ros::BRICSSelectPlanner::Response res;

	ROS_INFO("Ready to plan a path.");

//	ros::MultiThreadedSpinner spinner(2); // Use 2 threads
//	spinner.spin();
	ros::spin();

}
