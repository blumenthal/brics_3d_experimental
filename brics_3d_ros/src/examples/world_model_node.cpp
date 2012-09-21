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

#include "worldModel/sceneGraph/DotVisualizer.h"
#include "worldModel/sceneGraph/DotGraphGenerator.h"
#include "worldModel/sceneGraph/SceneGraphFacade.h"
#include "core/Logger.h"

#ifdef ENABLE_OSG
	#include <worldModel/sceneGraph/OSGVisualizer.h>
#endif

using BRICS_3D::Logger;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "world_model_node");
	ros::NodeHandle n;
	BRICS_3D::Logger::setMinLoglevel(BRICS_3D::Logger::LOGDEBUG);
	BRICS_3D::WorldModel* wm = new BRICS_3D::WorldModel();

#ifdef ENABLE_OSG
	// Visualization tool for world model
	BRICS_3D::RSG::OSGVisualizer* wmObserver = new BRICS_3D::RSG::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization
#endif

	BRICS_3D::RSG::DotVisualizer dbgObserver(&wm->scene);
//	wm->scene.attachUpdateObserver(&dbgObserver);

	BRICS_3D::RSG::SceneGraphROSCommunicator updater(n, "/worldModel/brics_mm/");
	wm->scene.attachUpdateObserver(&updater);

	BRICS_3D::WorldModelQueryServer wmServer(n, wm);
	wmServer.setServiceNameSpace("/worldModel/");
	wmServer.initialize();

	LOG(INFO) << "Ready.";

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
//	ros::spin();
	delete wm;
	return 0;
}



/* EOF */
