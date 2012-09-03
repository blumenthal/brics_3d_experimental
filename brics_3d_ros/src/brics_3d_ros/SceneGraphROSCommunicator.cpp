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

#include "SceneGraphROSCommunicator.h"
#include "core/Logger.h"

#include "SceneGraphTypeCasts.h"

using BRICS_3D::Logger;

namespace BRICS_3D {

namespace RSG {

SceneGraphROSCommunicator::SceneGraphROSCommunicator(ros::NodeHandle n, std::string nameSpace) {
	this->node = n;
	this->serviceNameSpace = nameSpace;
	initialize();
}

SceneGraphROSCommunicator::~SceneGraphROSCommunicator() {
	// TODO Auto-generated destructor stub
}

void SceneGraphROSCommunicator::initialize() {

//	/* ROS Queries */
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getRootId";
//	getRootIdClient = node.serviceClient<brics_3d_msgs::GetRootId>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getNodes";
//	getNodesClient = node.serviceClient<brics_3d_msgs::GetNodes>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getNodeAttributes";
//	getNodeAttributesClient = node.serviceClient<brics_3d_msgs::GetNodeAttributes>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getNodeParents";
//    getNodeParentsClient = node.serviceClient<brics_3d_msgs::GetNodeParents>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getGroupChildren";
//    getGroupChildrenClient = node.serviceClient<brics_3d_msgs::GetGroupChildren>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getTransform";
//    getTransformClient = node.serviceClient<brics_3d_msgs::GetTransform>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getGeometry";
//    getGeometryClient = node.serviceClient<brics_3d_msgs::GetGeometry>(serviceName.str());
//	serviceName.str("");
//	serviceName << serviceNameSpace << "getTransformForNode";
//    getTransformForNodeClient = node.serviceClient<brics_3d_msgs::GetTransformForNode>(serviceName.str());
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
	serviceName.str("");
	serviceName << serviceNameSpace << "addNode";
	addNodeCallbackClient = node.serviceClient<brics_3d_msgs::AddNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addGroup";
	addGroupCallbackClient = node.serviceClient<brics_3d_msgs::AddGroup>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addTransformNode";
	addTransformNodeCallbackClient = node.serviceClient<brics_3d_msgs::AddTransformNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addGeometricNode";
	addGeometricNodeClient = node.serviceClient<brics_3d_msgs::AddGeometricNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "setNodeAttributes";
    setNodeAttributesClient = node.serviceClient<brics_3d_msgs::SetNodeAttributes>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "setTransform";
    setTransformClient = node.serviceClient<brics_3d_msgs::SetTransform>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "deleteNode";
    deleteNodeClient = node.serviceClient<brics_3d_msgs::DeleteNode>(serviceName.str());
	serviceName.str("");
	serviceName << serviceNameSpace << "addParent";
    addParentClient = node.serviceClient<brics_3d_msgs::AddParent>(serviceName.str());

	brics_3d_msgs::AddNode addNodeUpdate;
	brics_3d_msgs::AddGroup addGroupUpdate;
	brics_3d_msgs::AddTransformNode addTransformNodeUpdate;
	brics_3d_msgs::AddGeometricNode addGeometricNodeUpdate;
	brics_3d_msgs::SetNodeAttributes setNodeAttributesUpdate;
	brics_3d_msgs::SetTransform setTransformUpdate;
	brics_3d_msgs::DeleteNode deleteNodeUpdate;
	brics_3d_msgs::AddParent addParentUpdate;

}

bool SceneGraphROSCommunicator::addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding Node";

	//TODO: check if alredy there?
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addNodeUpdate.request.attributes);
	addNodeUpdate.request.parentId = parentId;

	if (!addNodeCallbackClient.call(addNodeUpdate)) {
		LOG(ERROR) << "Failed to call service AddNode";
		return false;
	}

	assignedId = addNodeUpdate.response.assignedId;

	return true;
}

bool SceneGraphROSCommunicator::addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding Group";

	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addGroupUpdate.request.attributes);
	addGroupUpdate.request.parentId = parentId;

	if (!addGroupCallbackClient.call(addGroupUpdate)) {
		LOG(ERROR) << "Failed to call service AddGroup";
		return false;
	}

	assignedId = addGroupUpdate.response.assignedId;

	return true;
}

bool SceneGraphROSCommunicator::addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding TransformNode";

	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addTransformNodeUpdate.request.attributes);
	SceneGraphTypeCasts::convertTransformToRosMsg(transform, addTransformNodeUpdate.request.transform);
//	SceneGraphTypeCasts::convertTimeStampToRosMsg(); //FIXME
	addTransformNodeUpdate.request.parentId = parentId;
	addTransformNodeUpdate.request.stamp = ros::Time::now();

	if (!addTransformNodeCallbackClient.call(addTransformNodeUpdate)) {
		LOG(ERROR) << "Failed to call service AddTransformNode";
		return false;
	}

	assignedId = addTransformNodeUpdate.response.assignedId;

	return true;
}


bool SceneGraphROSCommunicator::addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding GeometricNode";

	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addGeometricNodeUpdate.request.attributes);
	SceneGraphTypeCasts::convertShapeToRosMsg(shape, addGeometricNodeUpdate.request.shape);

	addGeometricNodeUpdate.request.parentId = parentId;
	addGeometricNodeUpdate.request.stamp = ros::Time::now(); //FIXME

	if (!addGeometricNodeClient.call(addGeometricNodeUpdate)) {
		LOG(ERROR) << "Failed to call service AddGeometricNodeUpdate";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: setting Attributes";

	SceneGraphTypeCasts::convertAttributesToRosMsg(newAttributes, setNodeAttributesUpdate.request.newAttributes);
	setNodeAttributesUpdate.request.id = id;

	if (!setNodeAttributesClient.call(setNodeAttributesUpdate)) {
		LOG(ERROR) << "Failed to call service Set Node Attributes";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: updating Transform";

	SceneGraphTypeCasts::convertTransformToRosMsg(transform, setTransformUpdate.request.transform);

	setTransformUpdate.request.id = id;
	setTransformUpdate.request.stamp = ros::Time::now(); //FIXME

	if (!setTransformClient.call(setTransformUpdate)) {
		LOG(ERROR) << "Failed to call service Set Transform";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::deleteNode(unsigned int id) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: deleting Node";

	deleteNodeUpdate.request.id = id;

	if (!deleteNodeClient.call(deleteNodeUpdate)) {
		LOG(ERROR) << "Failed to call service Delet Node";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::addParent(unsigned int id, unsigned int parentId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding parent Node";

	addParentUpdate.request.id = id;
	addParentUpdate.request.parentId = parentId;

	if (!addParentClient.call(addParentUpdate)) {
		LOG(ERROR) << "Failed to call service Add Parent";
		return false;
	}

	return true;
}


}

}

/* EOF */