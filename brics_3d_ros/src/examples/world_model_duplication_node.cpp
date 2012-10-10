/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#include <iostream>
#include <fstream>

#include "../brics_3d_ros/WorldModelQueryServer.h"
#include "../brics_3d_ros/SceneGraphROSCommunicator.h"

#include "brics_3d/worldModel/sceneGraph/DotVisualizer.h"
#include "brics_3d/worldModel/sceneGraph/DotGraphGenerator.h"
#include "brics_3d/worldModel/sceneGraph/SceneGraphFacade.h"
#include "brics_3d/core/Logger.h"

#ifdef ENABLE_OSG
	#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#endif

#include "brics_3d_msgs/ResentSceneGraph.h"

int main(int argc, char **argv)
{

	std::stringstream serviceName;
	std::string serviceNameSpace = "/worldModel/";

	ros::init(argc, argv, "world_model_duplication_node");
	ros::NodeHandle n;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

#ifdef ENABLE_OSG
	// Visualization tool for world model
	brics_3d::rsg::OSGVisualizer* wmObserver = new brics_3d::rsg::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization
#endif

	brics_3d::rsg::DotVisualizer dbgObserver(&wm->scene);
	wm->scene.attachUpdateObserver(&dbgObserver);

//	brics_3d::rsg::SceneGraphROSCommunicator updater(n, "/worldModel/dev/null"); //loopback would create a deadlock
//	wm->scene.attachUpdateObserver(&updater);

	brics_3d::WorldModelQueryServer wmServer(n, wm);
//	wmServer.setServiceNameSpace("/worldModel/sampleListner/");
	wmServer.setServiceNameSpace("/worldModel/brics_mm/");
	wmServer.initialize();

	ros::AsyncSpinner spinner(2);
	spinner.start();

	/* intialize data */
	serviceName.str("");
	serviceName << serviceNameSpace << "resentSceneGraph";
	ros::ServiceClient resentSceneGraphClient = n.serviceClient<brics_3d_msgs::ResentSceneGraph>(serviceName.str());
	brics_3d_msgs::ResentSceneGraph resentSceneGraphRequest;
	resentSceneGraphRequest.request.subGraphRootId = wm->scene.getRootId();
	if (!resentSceneGraphClient.call(resentSceneGraphRequest)) {
		ROS_ERROR("Could not request to resent secene graph.");
	}

	ROS_INFO("Ready.");

	ros::spin();
	delete wm;
	return 0;
}



/* EOF */
