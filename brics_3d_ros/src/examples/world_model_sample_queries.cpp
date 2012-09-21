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

/* ROS includes */
#include "ros/ros.h"
#include "tf/tf.h"

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

int main(int argc, char **argv)
{

	ros::init(argc, argv, "world_modle_sample_queries");
	ros::NodeHandle n;

	std::stringstream serviceName;
	std::string serviceNameSpace = "/worldModel/";
//	std::string serviceNameSpace = "/worldModel/sampleListner/";

	/* ROS Queries */
	serviceName.str("");
	serviceName << serviceNameSpace << "getRootId";
	ros::ServiceClient getRootIdClient = n.serviceClient<brics_3d_msgs::GetRootId>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getNodes";
	ros::ServiceClient getNodesClient = n.serviceClient<brics_3d_msgs::GetNodes>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getNodeAttributes";
	ros::ServiceClient getNodeAttributesClient = n.serviceClient<brics_3d_msgs::GetNodeAttributes>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getNodeParents";
    ros::ServiceClient getNodeParentsClient = n.serviceClient<brics_3d_msgs::GetNodeParents>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getGroupChildren";
    ros::ServiceClient getGroupChildrenClient = n.serviceClient<brics_3d_msgs::GetGroupChildren>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getTransform";
    ros::ServiceClient getTransformClient = n.serviceClient<brics_3d_msgs::GetTransform>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getGeometry";
    ros::ServiceClient getGeometryClient = n.serviceClient<brics_3d_msgs::GetGeometry>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "getTransformForNode";
    ros::ServiceClient getTransformForNodeClient = n.serviceClient<brics_3d_msgs::GetTransformForNode>(serviceName.str());

	brics_3d_msgs::GetRootId getRootIDQuery;
	brics_3d_msgs::GetNodes getNodesQuery;
	brics_3d_msgs::GetNodeAttributes getNodeAttributesQuery;
	brics_3d_msgs::GetNodeParents getNodeParentsQuery;
	brics_3d_msgs::GetGroupChildren getGroupChildrenQuery;
	brics_3d_msgs::GetTransform getTransformQuery;
	brics_3d_msgs::GetGeometry getGeometryQuery;
	brics_3d_msgs::GetTransformForNode getTransformForNodeQuery;

	/* ROS Updates */
	serviceName.str("");
	serviceName << serviceNameSpace << "addNode";
	ros::ServiceClient addNodeCallbackClient = n.serviceClient<brics_3d_msgs::AddNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addGroup";
	ros::ServiceClient addGroupCallbackClient = n.serviceClient<brics_3d_msgs::AddGroup>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addTransformNode";
	ros::ServiceClient addTransformNodeCallbackClient = n.serviceClient<brics_3d_msgs::AddTransformNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addGeometricNode";
	ros::ServiceClient addGeometricNodeClient = n.serviceClient<brics_3d_msgs::AddGeometricNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "setNodeAttributes";
    ros::ServiceClient setNodeAttributesClient = n.serviceClient<brics_3d_msgs::SetNodeAttributes>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "setTransform";
    ros::ServiceClient setTransformClient = n.serviceClient<brics_3d_msgs::SetTransform>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "deleteNode";
    ros::ServiceClient deleteNodeClient = n.serviceClient<brics_3d_msgs::DeleteNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addParent";
    ros::ServiceClient addParentClient = n.serviceClient<brics_3d_msgs::AddParent>(serviceName.str());

	brics_3d_msgs::AddNode addNodeUpdate;
	brics_3d_msgs::AddGroup addGroupUpdate;
	brics_3d_msgs::AddTransformNode addTransformNodeUpdate;
	brics_3d_msgs::AddGeometricNode addGeometricNodeUpdate;
	brics_3d_msgs::SetNodeAttributes setNodeAttributesUpdate;
	brics_3d_msgs::SetTransform setTransformUpdate;
	brics_3d_msgs::DeleteNode deleteNodeUpdate;
	brics_3d_msgs::AddParent addParentUpdate;

	/*
	 * Do something:
	 */

	/* Get root ID */
	if (!getRootIdClient.call(getRootIDQuery)) {
		ROS_ERROR("Failed to call service GetRootId");
		return 1;
	}
	ROS_INFO("Root ID = %i", getRootIDQuery.response.rootId);

	/* Add a new group at root */
	addGroupUpdate.request.attributes.resize(1);
	addGroupUpdate.request.attributes[0].key = "name";
	addGroupUpdate.request.attributes[0].value = "some_group";
	addGroupUpdate.request.parentId = getRootIDQuery.response.rootId;

	if (!addGroupCallbackClient.call(addGroupUpdate)) {
		ROS_ERROR("Failed to call service AddGroup");
		return 1;
	}
	ROS_INFO("Added new group with ID %i", addGroupUpdate.response.assignedId);


	/* Add a new node at group */
	addNodeUpdate.request.attributes.resize(1);
	addNodeUpdate.request.attributes[0].key = "color";
	addNodeUpdate.request.attributes[0].value = "red";
	addNodeUpdate.request.parentId = addGroupUpdate.response.assignedId;//getRootIDQuery.response.rootId;

	if (!addNodeCallbackClient.call(addNodeUpdate)) {
		ROS_ERROR("Failed to call service AddNode");
		return 1;
	}
	ROS_INFO("Added new node with ID %i", addNodeUpdate.response.assignedId);


	/* Add a transform at group */
	addTransformNodeUpdate.request.attributes.resize(1);
	addTransformNodeUpdate.request.attributes[0].key = "tf";
	addTransformNodeUpdate.request.attributes[0].value = "some_tf";
	addTransformNodeUpdate.request.parentId = addGroupUpdate.response.assignedId;
	addTransformNodeUpdate.request.stamp = ros::Time::now();
	addTransformNodeUpdate.request.transform.transform.translation.x = 1;
	addTransformNodeUpdate.request.transform.transform.translation.y = 2;
	addTransformNodeUpdate.request.transform.transform.translation.z = 3;
	addTransformNodeUpdate.request.transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	if (!addTransformNodeCallbackClient.call(addTransformNodeUpdate)) {
		ROS_ERROR("Failed to call service AddTransformNode");
		return 1;
	}
	ROS_INFO("Added new transform with ID %i", addTransformNodeUpdate.response.assignedId);

	/* Add a new box. */
	addGeometricNodeUpdate.request.parentId = addTransformNodeUpdate.response.assignedId;
	addGeometricNodeUpdate.request.attributes.resize(1);
	addGeometricNodeUpdate.request.attributes[0].key = "name";
	addGeometricNodeUpdate.request.attributes[0].value = "some_box";
	addGeometricNodeUpdate.request.shape.type = arm_navigation_msgs::Shape::BOX;
	addGeometricNodeUpdate.request.shape.dimensions.resize(3);
	addGeometricNodeUpdate.request.shape.dimensions[0] = 0.2;
	addGeometricNodeUpdate.request.shape.dimensions[1] = 0.6;
	addGeometricNodeUpdate.request.shape.dimensions[2] = 1;
	addGeometricNodeUpdate.request.stamp = ros::Time::now();

	if (!addGeometricNodeClient.call(addGeometricNodeUpdate)) {
		ROS_ERROR("Failed to call service AddGeometricNodeUpdate");
		return 1;
	}
	ROS_INFO("Added new GeometricNode with ID %i", addGeometricNodeUpdate.response.assignedId);

	/* Add a new cylinder. */
	addGeometricNodeUpdate.request.parentId = addTransformNodeUpdate.response.assignedId;
	addGeometricNodeUpdate.request.attributes.resize(1);
	addGeometricNodeUpdate.request.attributes[0].key = "name";
	addGeometricNodeUpdate.request.attributes[0].value = "some_cylinder";
	addGeometricNodeUpdate.request.shape.type = arm_navigation_msgs::Shape::CYLINDER;
	addGeometricNodeUpdate.request.shape.dimensions.resize(2);
	addGeometricNodeUpdate.request.shape.dimensions[0] = 0.1;
	addGeometricNodeUpdate.request.shape.dimensions[1] = 1.5;
	addGeometricNodeUpdate.request.stamp = ros::Time::now();

	if (!addGeometricNodeClient.call(addGeometricNodeUpdate)) {
		ROS_ERROR("Failed to call service AddGeometricNodeUpdate");
		return 1;
	}
	ROS_INFO("Added new GeometricNode with ID %i", addGeometricNodeUpdate.response.assignedId);

	/* check how many nodes with certain attributes are stored */
	getNodesQuery.request.attributes.resize(1);
	getNodesQuery.request.attributes[0].key = "color";
	getNodesQuery.request.attributes[0].value = "red";

	if (!getNodesClient.call(getNodesQuery)) {
		ROS_ERROR("Failed to call service Get Node");
		return 1;
	}

	ROS_INFO("Found %i IDs.", getNodesQuery.response.ids.size());
	for (unsigned int i = 0; i < getNodesQuery.response.ids.size(); ++i) {
		ROS_INFO("	ID = %i", getNodesQuery.response.ids[i]);
	}

	return 0;

	ros::spin();
	return 0;
}



/* EOF */
