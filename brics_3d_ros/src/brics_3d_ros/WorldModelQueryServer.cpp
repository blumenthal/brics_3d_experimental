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

#include "WorldModelQueryServer.h"
#include "SceneGraphTypeCasts.h"

#include "brics_3d/core/Logger.h"

using brics_3d::Logger;
using namespace brics_3d::rsg;

namespace brics_3d {

WorldModelQueryServer::WorldModelQueryServer(ros::NodeHandle n, brics_3d::WorldModel* wm) :
		node(n) {
	this->wm = wm;
	serviceNameSpace = "/worldModel/"; // default
//	initialize();
}

WorldModelQueryServer::~WorldModelQueryServer() {

}

void WorldModelQueryServer::initialize() {
	std::stringstream serviceName;
//	serviceNameSpace = "/worldModel/";

	/* Queries */

	serviceName.str("");
	serviceName << serviceNameSpace << "getRootId";
	getRootIdService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getRootIdCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getNodes";
	getNodesService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getNodesCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getNodeAttributes";
	getNodeAttributesService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getNodeAttributesCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getNodeParents";
	getNodeParentsService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getNodeParentsCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getGroupChildren";
	getGroupChildrenService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getGroupChildrenCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getTransform";
	getTransformService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getTransformCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getGeometry";
	getGeometryService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getGeometryCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "getTransformForNode";
	getTransformForNodeService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::getTransformForNodeCallback, this);

	/* Updates */

	serviceName.str("");
	serviceName << serviceNameSpace << "addNode";
	addNodeService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::addNodeCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "addGroup";
	addGroupService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::addGroupCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "addTransformNode";
	addTransformNodeService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::addTransformNodeCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "addGeometricNode";
	addGeometricNodeService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::addGeometricNodeCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "setNodeAttributes";
	setNodeAttributesService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::setNodeAttributesCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "setTransform";
	setTransformService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::setTransformCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "deleteNode";
	deleteNodeService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::deleteNodeCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "addParent";
	addParentService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::addParentCallback, this);

	serviceName.str("");
	serviceName << serviceNameSpace << "removeParent";
	addParentService =  node.advertiseService(serviceName.str(), &WorldModelQueryServer::removeParentCallback, this);
}

/*
 * Query callbacks
 */

bool WorldModelQueryServer::getRootIdCallback(brics_3d_msgs::GetRootId::Request& request, brics_3d_msgs::GetRootId::Response& response) {
	response.rootId = wm->scene.getRootId();
	return true;
}

bool WorldModelQueryServer::getNodesCallback(brics_3d_msgs::GetNodes::Request& request, brics_3d_msgs::GetNodes::Response& response) {
	vector<brics_3d::rsg::Attribute> attributes;
	vector<unsigned int> ids;
	SceneGraphTypeCasts::convertRosMsgToAttributes(request.attributes, attributes);
	response.succeeded = wm->scene.getNodes(attributes, ids);
	SceneGraphTypeCasts::convertIdsToRosMsg(ids, response.ids);
	return response.succeeded;
}

bool WorldModelQueryServer::getNodeAttributesCallback(brics_3d_msgs::GetNodeAttributes::Request& request, brics_3d_msgs::GetNodeAttributes::Response& response){
	vector<brics_3d::rsg::Attribute> attributes;
	response.succeeded = wm->scene.getNodeAttributes(request.id, attributes);
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, response.attributes);
	return response.succeeded;
}

bool WorldModelQueryServer::getNodeParentsCallback(brics_3d_msgs::GetNodeParents::Request& request, brics_3d_msgs::GetNodeParents::Response& response) {
	vector<unsigned int> parentIds;
	response.succeeded = wm->scene.getNodeParents(request.id, parentIds);
	SceneGraphTypeCasts::convertIdsToRosMsg(parentIds, response.parentIds);
	return response.succeeded;
}

bool WorldModelQueryServer::getGroupChildrenCallback(brics_3d_msgs::GetGroupChildren::Request& request, brics_3d_msgs::GetGroupChildren::Response& response) {
	vector<unsigned int> childIds;

	response.succeeded = wm->scene.getGroupChildren(request.id, childIds);

	SceneGraphTypeCasts::convertIdsToRosMsg(childIds, response.childIds);

	return response.succeeded;
}

bool WorldModelQueryServer::getTransformCallback(brics_3d_msgs::GetTransform::Request& request, brics_3d_msgs::GetTransform::Response& response) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
	TimeStamp timeStamp;

	SceneGraphTypeCasts::convertRosMsgToTimeStamp(request.stamp, timeStamp);
	response.succeeded = wm->scene.getTransform(request.id, timeStamp, transform);
	SceneGraphTypeCasts::convertTransformToRosMsg(transform, response.transform);

	return response.succeeded;
}

bool WorldModelQueryServer::getGeometryCallback(brics_3d_msgs::GetGeometry::Request& request, brics_3d_msgs::GetGeometry::Response& response) {
	Shape::ShapePtr shape;
	TimeStamp timeStamp;

	response.succeeded = wm->scene.getGeometry(request.id, shape, timeStamp);
	SceneGraphTypeCasts::convertShapeToRosMsg(shape, response.shape);
	//TODO cast timestamp

	return response.succeeded;
}

bool WorldModelQueryServer::getTransformForNodeCallback(brics_3d_msgs::GetTransformForNode::Request& request, brics_3d_msgs::GetTransformForNode::Response& response) {
	TimeStamp timeStamp;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform;

	SceneGraphTypeCasts::convertRosMsgToTimeStamp(request.stamp, timeStamp);
	response.succeeded = wm->scene.getTransformForNode(request.id, request.idReferenceNode, timeStamp, transform);
	SceneGraphTypeCasts::convertTransformToRosMsg(transform, response.transform);

	return response.succeeded;
}



/*
 * Update callbacks
 */

bool WorldModelQueryServer::addNodeCallback(brics_3d_msgs::AddNode::Request& request, brics_3d_msgs::AddNode::Response& response) {
	vector<brics_3d::rsg::Attribute> attributes;
	unsigned int assignedId;
	assignedId = request.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.

	SceneGraphTypeCasts::convertRosMsgToAttributes(request.attributes, attributes);
	response.succeeded = wm->scene.addNode(request.parentId, assignedId, attributes, request.forcedId);

	response.assignedId = assignedId;
	return response.succeeded;
}

bool WorldModelQueryServer::addGroupCallback(brics_3d_msgs::AddGroup::Request& request, brics_3d_msgs::AddGroup::Response& response) {
	unsigned int assignedId;
	vector<brics_3d::rsg::Attribute> attributes;
	assignedId = request.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.


	SceneGraphTypeCasts::convertRosMsgToAttributes(request.attributes, attributes);
	response.succeeded = wm->scene.addGroup(request.parentId, assignedId, attributes, request.forcedId);

	response.assignedId = assignedId;
	return response.succeeded;
}

bool WorldModelQueryServer::addTransformNodeCallback(brics_3d_msgs::AddTransformNode::Request& request, brics_3d_msgs::AddTransformNode::Response& response) {
	unsigned int assignedId;
	vector<brics_3d::rsg::Attribute> attributes;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
	TimeStamp timeStamp;
	assignedId = request.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.


	SceneGraphTypeCasts::convertRosMsgToAttributes(request.attributes, attributes);
	SceneGraphTypeCasts::convertRosMsgToTransform(request.transform, transform);
	SceneGraphTypeCasts::convertRosMsgToTimeStamp(request.stamp, timeStamp);
	response.succeeded = wm->scene.addTransformNode(request.parentId, assignedId, attributes, transform, timeStamp, request.forcedId);

	response.assignedId = assignedId;
	return response.succeeded;
}

bool WorldModelQueryServer::addGeometricNodeCallback(brics_3d_msgs::AddGeometricNode::Request& request, brics_3d_msgs::AddGeometricNode::Response& response) {
	unsigned int assignedId;
	vector<brics_3d::rsg::Attribute> attributes;
	Shape::ShapePtr shape;
	TimeStamp timeStamp;
	assignedId = request.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.

	LOG(DEBUG) << "WorldModelQueryServer: adding GeometricNode with parent ID: " << request.parentId;

	SceneGraphTypeCasts::convertRosMsgToAttributes(request.attributes, attributes);
	SceneGraphTypeCasts::convertRosMsgToShape(request.shape, shape);
//	rsg::Box::BoxPtr box(new rsg::Box());
//	box =  boost::dynamic_pointer_cast<rsg::Box>(shape);
//	assert(box!=0);
	SceneGraphTypeCasts::convertRosMsgToTimeStamp(request.stamp, timeStamp);
	response.succeeded = wm->scene.addGeometricNode(request.parentId, assignedId, attributes, shape, timeStamp, request.forcedId);

	response.assignedId = assignedId;
	return response.succeeded;
}

bool WorldModelQueryServer::setNodeAttributesCallback(brics_3d_msgs::SetNodeAttributes::Request& request, brics_3d_msgs::SetNodeAttributes::Response& response) {
	vector<brics_3d::rsg::Attribute> newAttributes;

	SceneGraphTypeCasts::convertRosMsgToAttributes(request.newAttributes, newAttributes);
	response.succeeded = wm->scene.setNodeAttributes(request.id, newAttributes);

	return response.succeeded;
}

bool WorldModelQueryServer::setTransformCallback(brics_3d_msgs::SetTransform::Request& request, brics_3d_msgs::SetTransform::Response& response) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
	TimeStamp timeStamp;

	SceneGraphTypeCasts::convertRosMsgToTransform(request.transform, transform);
	SceneGraphTypeCasts::convertRosMsgToTimeStamp(request.stamp, timeStamp);
//	LOG(DEBUG) << "setTransformCallback: new transform: at time" << request.stamp << std::endl << *transform;
	response.succeeded = wm->scene.setTransform(request.id ,transform, timeStamp);

	return response.succeeded;
}

bool WorldModelQueryServer::deleteNodeCallback(brics_3d_msgs::DeleteNode::Request& request, brics_3d_msgs::DeleteNode::Response& response) {
	response.succeeded = wm->scene.deleteNode(request.id);
	return response.succeeded;
}

bool WorldModelQueryServer::addParentCallback(brics_3d_msgs::AddParent::Request& request, brics_3d_msgs::AddParent::Response& response) {
	response.succeeded = wm->scene.addParent(request.id, request.parentId);
	return response.succeeded;
}

bool WorldModelQueryServer::removeParentCallback(brics_3d_msgs::RemoveParent::Request& request, brics_3d_msgs::RemoveParent::Response& response) {
	response.succeeded = wm->scene.removeParent(request.id, request.parentId);
	return response.succeeded;
}

}

/* EOF */
