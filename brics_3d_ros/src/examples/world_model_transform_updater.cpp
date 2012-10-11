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

	ros::init(argc, argv, "world_model_transform_updater");
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

	double x = 1.0;
	double y = 2.0;
	double z = 3.0;
	double increment = 0.01;

	/*
	 * Do something:
	 */

	int numberOfLoops = 10000;
	for (int loops = 0; loops < numberOfLoops; ++loops) {

		/* Get root ID */
		if (!getRootIdClient.call(getRootIDQuery)) {
			ROS_ERROR("Failed to call service GetRootId");
			return 1;
		}
		ROS_INFO("Root ID = %i", getRootIDQuery.response.rootId);

		/* check how many nodes with certain attributes are stored */
		getNodesQuery.request.attributes.resize(1);
		getNodesQuery.request.attributes[0].key = "tf";
		getNodesQuery.request.attributes[0].value = "some_tf";
//		getNodesQuery.request.attributes[0].key = "name";
//		getNodesQuery.request.attributes[0].value = "robot_to_sensor_tf";
//		getNodesQuery.request.attributes[0].value = "world_to_robot_tf";


		if (!getNodesClient.call(getNodesQuery)) {
			ROS_ERROR("Failed to call service Get Node");
			return 1;
		}




		ROS_INFO("Found %i IDs.", getNodesQuery.response.ids.size());
		for (unsigned int i = 0; i < getNodesQuery.response.ids.size(); ++i) {
			ROS_INFO("	ID = %i", getNodesQuery.response.ids[i]);

			getTransformQuery.request.id = getNodesQuery.response.ids[i];
			getTransformQuery.request.stamp = ros::Time::now(); //latest

			if (!getTransformClient.call(getTransformQuery)) {
				ROS_ERROR("Failed to call service Get Transform");
				continue;
			}

			x = getTransformQuery.response.transform.transform.translation.x;
			y = getTransformQuery.response.transform.transform.translation.y;
			z = getTransformQuery.response.transform.transform.translation.z;

			x = x + increment;
			y = y + increment;
			z = z; //+ increment;

			ROS_INFO("Setting to translation to (%f %f %f)", x,y,z);

			setTransformUpdate.request.id = getNodesQuery.response.ids[i];
			setTransformUpdate.request.stamp = ros::Time::now();
			setTransformUpdate.request.transform.transform.translation.x = x;
			setTransformUpdate.request.transform.transform.translation.y = y;
			setTransformUpdate.request.transform.transform.translation.z = z;
//			setTransformUpdate.request.transform.transform.rotation = tf::createQuaternionMsgFromYaw(x /*0*/);
			setTransformUpdate.request.transform.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, x,0);

			if (!setTransformClient.call(setTransformUpdate)) {
				ROS_ERROR("Failed to call service Set Transform");
				continue;
			}

		}
	}

	return 0;

	ros::spin();
	return 0;
}



/* EOF */
