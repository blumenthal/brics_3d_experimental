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

#ifndef SCENEGRAPHRESENTSERVER_H_
#define SCENEGRAPHRESENTSERVER_H_

#include <string>

/* ROS includes */
#include "ros/ros.h"

#include "brics_3d_msgs/ResentSceneGraph.h"

/* BRICS_3D includes */
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/SceneGraphToUpdatesTraverser.h>


namespace brics_3d {

namespace rsg {

class SceneGraphResentServer {
public:

	/**
	 * @brief Constructor with a ROS handle.
	 * @param n ROS handle
	 * @param wm Handle the the world model. All ROS service calls will be forwarded to this one.
	 */
	SceneGraphResentServer(ros::NodeHandle n, brics_3d::WorldModel* wm, brics_3d::rsg::ISceneGraphUpdate* updatesRecieverHandle);

	/**
	 * @brief Default destructor.
	 */
	virtual ~SceneGraphResentServer();

	/**
	 * @brief Bring up all services.
	 */
	void initialize();

    std::string getServiceNameSpace() const
    {
        return serviceNameSpace;
    }

    void setServiceNameSpace(std::string serviceNameSpace)
    {
        this->serviceNameSpace = serviceNameSpace;
    }

    /* service callbacks */
    bool resentSceneGraphCallback(brics_3d_msgs::ResentSceneGraph::Request& request, brics_3d_msgs::ResentSceneGraph::Response& response);

private:

	/// The ROS node handle.
	ros::NodeHandle node;

	/// Handle to scene graph based world model.
	brics_3d::WorldModel* wm;

	/**
	 * This is the reciever of all updates.
	 * Most likely it will be some ROS communicator.
	 */
	brics_3d::rsg::ISceneGraphUpdate* updatesRecieverHandle;

	/// Common namespace for all services.
	std::string serviceNameSpace;

	/// Server for the resent service.
    ros::ServiceServer resentSceneGraphService;

};

}

}

#endif /* SCENEGRAPHRESENTSERVER_H_ */

/* EOF */
