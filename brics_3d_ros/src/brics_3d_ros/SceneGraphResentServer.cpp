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

#include "SceneGraphResentServer.h"

namespace brics_3d {

namespace rsg {

SceneGraphResentServer::SceneGraphResentServer(ros::NodeHandle n, brics_3d::WorldModel* wm, brics_3d::rsg::ISceneGraphUpdate* updatesRecieverHandle) {
		this->node = n;
		this->wm = wm;
		this->updatesRecieverHandle = updatesRecieverHandle;
		serviceNameSpace = "/worldModel/"; // default
	}

SceneGraphResentServer::~SceneGraphResentServer() {

}

void SceneGraphResentServer::initialize() {
	std::stringstream serviceName;

	serviceName.str("");
	serviceName << serviceNameSpace << "resentSceneGraph";
	resentSceneGraphService = node.advertiseService(serviceName.str(), &SceneGraphResentServer::resentSceneGraphCallback, this);

}

bool SceneGraphResentServer::resentSceneGraphCallback(brics_3d_msgs::ResentSceneGraph::Request& request, brics_3d_msgs::ResentSceneGraph::Response& response) {
	ROS_INFO("Resending scene graph.");

	unsigned int subGraphRootId = request.subGraphRootId;
	SceneGraphToUpdatesTraverser graphSender(updatesRecieverHandle);
	wm->scene.executeGraphTraverser(&graphSender, subGraphRootId);

	ROS_INFO("Resending scene graph done.");

	return true;
}


}

}

/* EOF */
