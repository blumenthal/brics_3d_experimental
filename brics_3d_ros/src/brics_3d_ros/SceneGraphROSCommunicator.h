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


#ifndef SCENEGRAPHROSCOMMUNICATOR_H_
#define SCENEGRAPHROSCOMMUNICATOR_H_

#include "worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "worldModel/sceneGraph/PointCloud.h"
#include "worldModel/sceneGraph/Mesh.h"
#include "worldModel/sceneGraph/Box.h"
#include "worldModel/sceneGraph/Cylinder.h"
#include "worldModel/sceneGraph/Attribute.h"

/* ROS includes */
#include "ros/ros.h"

#include "brics_3d_msgs/AddNode.h"
#include "brics_3d_msgs/AddGroup.h"
#include "brics_3d_msgs/AddTransformNode.h"
#include "brics_3d_msgs/AddGeometricNode.h"
#include "brics_3d_msgs/SetNodeAttributes.h"
#include "brics_3d_msgs/SetTransform.h"
#include "brics_3d_msgs/DeleteNode.h"
#include "brics_3d_msgs/AddParent.h"

namespace BRICS_3D {

namespace RSG {

class SceneGraphROSCommunicator : public ISceneGraphUpdateObserver {
public:
	SceneGraphROSCommunicator(ros::NodeHandle n, std::string nameSpace);
	virtual ~SceneGraphROSCommunicator();

	/* implemetntations of observer interface */
	bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes);
	bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes);
	bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
	bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp);
	bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes);
	bool setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
	bool deleteNode(unsigned int id);
	bool addParent(unsigned int id, unsigned int parentId);

protected:

	void initialize();

	ros::NodeHandle node;

	std::stringstream serviceName;
	std::string serviceNameSpace;

//	/* ROS Queries */
//	ros::ServiceClient getRootIdClient;
//	ros::ServiceClient getNodesClient;
//	ros::ServiceClient getNodeAttributesClient;
//    ros::ServiceClient getNodeParentsClient;
//    ros::ServiceClient getGroupChildrenClient;
//    ros::ServiceClient getTransformClient;
//    ros::ServiceClient getGeometryClient;
//    ros::ServiceClient getTransformForNodeClient;
//
//	brics_3d_msgs::GetRootId getRootIDQuery;
//	brics_3d_msgs::GetNodes getNodesQuery;
//	brics_3d_msgs::GetNodeAttributes getNodeAttributesQuery;
//	brics_3d_msgs::GetNodeParents getNodeParentsQuery;
//	brics_3d_msgs::GetGroupChildren getGroupChildrenQuery;
//	brics_3d_msgs::GetTransform getTransformQuery;
//	brics_3d_msgs::GetGeometry getGeometryQuery;
//	brics_3d_msgs::GetTransformForNode getTransformForNodeQuery;

	/* ROS Updates */
	ros::ServiceClient addNodeCallbackClient;
	ros::ServiceClient addGroupCallbackClient;
	ros::ServiceClient addTransformNodeCallbackClient;
	ros::ServiceClient addGeometricNodeClient;
    ros::ServiceClient setNodeAttributesClient;
    ros::ServiceClient setTransformClient;
    ros::ServiceClient deleteNodeClient;
    ros::ServiceClient addParentClient;

	brics_3d_msgs::AddNode addNodeUpdate;
	brics_3d_msgs::AddGroup addGroupUpdate;
	brics_3d_msgs::AddTransformNode addTransformNodeUpdate;
	brics_3d_msgs::AddGeometricNode addGeometricNodeUpdate;
	brics_3d_msgs::SetNodeAttributes setNodeAttributesUpdate;
	brics_3d_msgs::SetTransform setTransformUpdate;
	brics_3d_msgs::DeleteNode deleteNodeUpdate;
	brics_3d_msgs::AddParent addParentUpdate;
};

}

}

#endif /* SCENEGRAPHROSCOMMUNICATOR_H_ */

/* EOF */
