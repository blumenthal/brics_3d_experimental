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

#ifndef WORLDMODELQUERYSERVER_H_
#define WORLDMODELQUERYSERVER_H_

/* ROS includes */
#include "ros/ros.h"

/* BRICS_3D includes */
#include <worldModel/WorldModel.h>

/* BRICS_3D <-> ROS types */
#include "brics_3d_msgs/GetRootId.h"
#include "brics_3d_msgs/GetNodes.h"
#include "brics_3d_msgs/GetNodeAttributes.h"
#include "brics_3d_msgs/GetNodeParents.h"
#include "brics_3d_msgs/GetGroupChildren.h"
#include "brics_3d_msgs/GetTransform.h"
#include "brics_3d_msgs/GetGeometry.h"
#include "brics_3d_msgs/GetTransformForNode.h"

#include "brics_3d_msgs/AddNode.h"
#include "brics_3d_msgs/AddGroup.h"
#include "brics_3d_msgs/AddTransformNode.h"
#include "brics_3d_msgs/AddGeometricNode.h"
#include "brics_3d_msgs/SetNodeAttributes.h"
#include "brics_3d_msgs/SetTransform.h"
#include "brics_3d_msgs/DeleteNode.h"
#include "brics_3d_msgs/AddParent.h"

namespace BRICS_3D {

/**
 * @brief ROS Wrapper (services) for a BRICS_3D::WorldModel handle.
 */
class WorldModelQueryServer {
public:

	/**
	 * @brief Constructor with a ROS handle.
	 * @param n ROS handle
	 * @param wm Handle the the worldmodel. All ROS service calls will be forwarded to this one.
	 */
	WorldModelQueryServer(ros::NodeHandle n, BRICS_3D::WorldModel* wm);

	/**
	 * @brief Default constructor.
	 */
	virtual ~WorldModelQueryServer();

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

    /* query interfaces callbacks */
    bool getRootIdCallback(brics_3d_msgs::GetRootId::Request& request, brics_3d_msgs::GetRootId::Response& response);
    bool getNodesCallback(brics_3d_msgs::GetNodes::Request& request, brics_3d_msgs::GetNodes::Response& response);
    bool getNodeAttributesCallback(brics_3d_msgs::GetNodeAttributes::Request& request, brics_3d_msgs::GetNodeAttributes::Response& response);
    bool getNodeParentsCallback(brics_3d_msgs::GetNodeParents::Request& request, brics_3d_msgs::GetNodeParents::Response& response);
    bool getGroupChildrenCallback(brics_3d_msgs::GetGroupChildren::Request& request, brics_3d_msgs::GetGroupChildren::Response& response);
    bool getTransformCallback(brics_3d_msgs::GetTransform::Request& request, brics_3d_msgs::GetTransform::Response& response);
    bool getGeometryCallback(brics_3d_msgs::GetGeometry::Request& request, brics_3d_msgs::GetGeometry::Response& response);
    bool getTransformForNodeCallback(brics_3d_msgs::GetTransformForNode::Request& request, brics_3d_msgs::GetTransformForNode::Response& response);

    /* update interfaces callbacks */
    bool addNodeCallback(brics_3d_msgs::AddNode::Request& request, brics_3d_msgs::AddNode::Response& response);
    bool addGroupCallback(brics_3d_msgs::AddGroup::Request& request, brics_3d_msgs::AddGroup::Response& response);
    bool addTransformNodeCallback(brics_3d_msgs::AddTransformNode::Request& request, brics_3d_msgs::AddTransformNode::Response& response);
    bool addGeometricNodeCallback(brics_3d_msgs::AddGeometricNode::Request& request, brics_3d_msgs::AddGeometricNode::Response& response);
    bool setNodeAttributesCallback(brics_3d_msgs::SetNodeAttributes::Request& request, brics_3d_msgs::SetNodeAttributes::Response& response);
    bool setTransformCallback(brics_3d_msgs::SetTransform::Request& request, brics_3d_msgs::SetTransform::Response& response);
    bool deleteNodeCallback(brics_3d_msgs::DeleteNode::Request& request, brics_3d_msgs::DeleteNode::Response& response);
    bool addParentCallback(brics_3d_msgs::AddParent::Request& request, brics_3d_msgs::AddParent::Response& response);


private:

	/// The ROS node handle.
	ros::NodeHandle node;

	/// ROS timestamp.
	ros::Time currentTime;

	/// Handle to scene graph based world model.
	BRICS_3D::WorldModel* wm;

	/// Common namespace for all services.
	std::string serviceNameSpace;

    /* query interfaces */
	ros::ServiceServer getRootIdService;
    ros::ServiceServer getNodesService;
    ros::ServiceServer getNodeAttributesService;
    ros::ServiceServer getNodeParentsService;
    ros::ServiceServer getGroupChildrenService;
    ros::ServiceServer getTransformService;
    ros::ServiceServer getGeometryService;
    ros::ServiceServer getTransformForNodeService;

    /* update interfaces */
    ros::ServiceServer addNodeService;
    ros::ServiceServer addGroupService;
    ros::ServiceServer addTransformNodeService;
    ros::ServiceServer addGeometricNodeService;
    ros::ServiceServer setNodeAttributesService;
    ros::ServiceServer setTransformService;
    ros::ServiceServer deleteNodeService;
    ros::ServiceServer addParentService;

};

}

#endif /* WORLDMODELQUERYSERVER_H_ */

/* EOF */
