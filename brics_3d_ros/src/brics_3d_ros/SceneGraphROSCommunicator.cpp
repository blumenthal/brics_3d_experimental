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
#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/SubGraphChecker.h"

#include "SceneGraphTypeCasts.h"

using brics_3d::Logger;
using namespace brics_3d::rsg;

namespace brics_3d {

namespace rsg {

SceneGraphROSCommunicator::SceneGraphROSCommunicator(ros::NodeHandle n, std::string nameSpace) {
	this->node = n;
	this->serviceNameSpace = nameSpace;
	initialize();
}

SceneGraphROSCommunicator::~SceneGraphROSCommunicator() {

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

	LOG(DEBUG) << "SceneGraphROSCommunicator: Initialization done.";

}

bool SceneGraphROSCommunicator::addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding Node";

	//TODO: check if alredy there?
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addNodeUpdate.request.attributes);
//	addNodeUpdate.request.parentId = parentId;
	SceneGraphTypeCasts::convertIdToRosMsg(parentId, addNodeUpdate.request.parentId);

	/* We want to enforce generation of a new node with exactly this assignedId ID.
	 * Otherwise subsequent syncs would fail due to inconsistend IDs...
	 */
//	addNodeUpdate.request.assignedId = assignedId;
	SceneGraphTypeCasts::convertIdToRosMsg(assignedId, addNodeUpdate.request.assignedId);
	addNodeUpdate.request.forcedId = true;

	if (!addNodeCallbackClient.call(addNodeUpdate)) {
		LOG(ERROR) << "Failed to call service AddNode";
		return false;
	}

	Id remoteAssignedId;
	SceneGraphTypeCasts::convertRosMsgToId(addNodeUpdate.response.assignedId, remoteAssignedId);
	if (assignedId != remoteAssignedId) { //Should be the same as we use forced IDs
		LOG(WARNING) << "SceneGraphROSCommunicator: Inconsistency of IDs detected. Should be: " << assignedId << " but is: " << remoteAssignedId;
	}
	assignedId = remoteAssignedId;


	return true;
}

bool SceneGraphROSCommunicator::addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding Group";

	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addGroupUpdate.request.attributes);
//	addGroupUpdate.request.parentId = parentId;
	SceneGraphTypeCasts::convertIdToRosMsg(parentId, addGroupUpdate.request.parentId);

	/* We want to enforce generation of a new node with exactly this assignedId ID.
	 * Otherwise subsequent syncs would fail due to inconsistend IDs...
	 */
	SceneGraphTypeCasts::convertIdToRosMsg(assignedId, addGroupUpdate.request.assignedId);
	addGroupUpdate.request.forcedId = true;

	if (!addGroupCallbackClient.call(addGroupUpdate)) {
		LOG(ERROR) << "Failed to call service AddGroup";
		return false;
	}

	Id remoteAssignedId;
	SceneGraphTypeCasts::convertRosMsgToId(addGroupUpdate.response.assignedId, remoteAssignedId);
	if (assignedId != remoteAssignedId) { //Should be the same as we use forced IDs
		LOG(WARNING) << "SceneGraphROSCommunicator: Inconsistency of IDs detected. Should be: " << assignedId << " but is: " << remoteAssignedId;
	}
	assignedId = addGroupUpdate.response.assignedId;

	return true;
}

bool SceneGraphROSCommunicator::addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding TransformNode";

	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addTransformNodeUpdate.request.attributes);
	SceneGraphTypeCasts::convertTransformToRosMsg(transform, addTransformNodeUpdate.request.transform);
//	SceneGraphTypeCasts::convertTimeStampToRosMsg(); //FIXME
//	addTransformNodeUpdate.request.parentId = parentId;
	SceneGraphTypeCasts::convertIdToRosMsg(parentId, addTransformNodeUpdate.request.parentId);
	addTransformNodeUpdate.request.stamp = ros::Time::now();

	/* We want to enforce generation of a new node with exactly this assignedId ID.
	 * Otherwise subsequent syncs would fail due to inconsistend IDs...
	 */
	SceneGraphTypeCasts::convertIdToRosMsg(assignedId, addTransformNodeUpdate.request.assignedId);
	addTransformNodeUpdate.request.forcedId = true;

	if (!addTransformNodeCallbackClient.call(addTransformNodeUpdate)) {
		LOG(ERROR) << "Failed to call service AddTransformNode";
		return false;
	}

	Id remoteAssignedId;
	SceneGraphTypeCasts::convertRosMsgToId(addTransformNodeUpdate.response.assignedId, remoteAssignedId);
	if (assignedId != remoteAssignedId) { //Should be the same as we use forced IDs
		LOG(WARNING) << "SceneGraphROSCommunicator: Inconsistency of IDs detected. Should be: " << assignedId << " but is: " << remoteAssignedId;
	}
	assignedId = addTransformNodeUpdate.response.assignedId;

	return true;
}

bool SceneGraphROSCommunicator::addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId) {
	LOG(ERROR) << "SceneGraphROSCommunicator: adding UncertainTransformNode - this is not yet implemented!";
	return false;
}

bool SceneGraphROSCommunicator::addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding GeometricNode with parent ID: " << parentId;

	bool noSharing = false;
	Attribute noSharingTag("rsgInfo","non_shared");
	noSharing = attributeListContainsAttribute(attributes, noSharingTag);

	if (noSharing) {
		LOG(DEBUG) << "SceneGraphROSCommunicator: GeometricNode has non_shared tag. Skipping it.";
//		brics_3d::rsg::Box::BoxPtr dummyBox(new brics_3d::rsg::Box());
//		dummyBox->setSizeX(0.01);
//		dummyBox->setSizeY(0.01);
//		dummyBox->setSizeZ(0.01);
//		shape = boost::dynamic_pointer_cast<rsg::Shape>(dummyBox);
		return true;
	}



	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, addGeometricNodeUpdate.request.attributes);
	if(!SceneGraphTypeCasts::convertShapeToRosMsg(shape, addGeometricNodeUpdate.request.shape)) {
		return false;
	}

//	addGeometricNodeUpdate.request.parentId = parentId;
	SceneGraphTypeCasts::convertIdToRosMsg(parentId, addGeometricNodeUpdate.request.parentId);
	addGeometricNodeUpdate.request.stamp = ros::Time::now(); //FIXME

	/* We want to enforce generation of a new node with exactly this assignedId ID.
	 * Otherwise subsequent syncs would fail due to inconsistend IDs...
	 */
	SceneGraphTypeCasts::convertIdToRosMsg(assignedId, addGeometricNodeUpdate.request.assignedId);
	addGeometricNodeUpdate.request.forcedId = true;

	if (!addGeometricNodeClient.call(addGeometricNodeUpdate)) {
		LOG(ERROR) << "Failed to call service AddGeometricNodeUpdate";
		return false;
	}

	Id remoteAssignedId;
	SceneGraphTypeCasts::convertRosMsgToId(addGeometricNodeUpdate.response.assignedId, remoteAssignedId);
	if (assignedId != remoteAssignedId) { //Should be the same as we use forced IDs
		LOG(WARNING) << "SceneGraphROSCommunicator: Inconsistency of IDs detected. Should be: " << assignedId << " but is: " << remoteAssignedId;
	}
	return true;
}

bool SceneGraphROSCommunicator::setNodeAttributes(Id id, vector<Attribute> newAttributes) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: setting Attributes";

	SceneGraphTypeCasts::convertAttributesToRosMsg(newAttributes, setNodeAttributesUpdate.request.newAttributes);
//	setNodeAttributesUpdate.request.id = id;
	SceneGraphTypeCasts::convertIdToRosMsg(id, setNodeAttributesUpdate.request.id);

	if (!setNodeAttributesClient.call(setNodeAttributesUpdate)) {
		LOG(ERROR) << "Failed to call service Set Node Attributes";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: updating Transform";

	SceneGraphTypeCasts::convertTransformToRosMsg(transform, setTransformUpdate.request.transform);

//	setTransformUpdate.request.id = id;
	SceneGraphTypeCasts::convertIdToRosMsg(id, setTransformUpdate.request.id);
	setTransformUpdate.request.stamp = ros::Time::now(); //FIXME

	if (!setTransformClient.call(setTransformUpdate)) {
		LOG(ERROR) << "Failed to call service Set Transform";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp) {
	LOG(ERROR) << "SceneGraphROSCommunicator: updating UncertainTransformNode - this is not yet implemented!";
	return false;
}

bool SceneGraphROSCommunicator::deleteNode(Id id) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: deleting Node";

//	deleteNodeUpdate.request.id = id;
	SceneGraphTypeCasts::convertIdToRosMsg(id, deleteNodeUpdate.request.id);

	if (!deleteNodeClient.call(deleteNodeUpdate)) {
		LOG(ERROR) << "Failed to call service Delete Node";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::addParent(Id id, Id parentId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: adding parent Node";

//	addParentUpdate.request.id = id;
//	addParentUpdate.request.parentId = parentId;
	SceneGraphTypeCasts::convertIdToRosMsg(id, addParentUpdate.request.id);
	SceneGraphTypeCasts::convertIdToRosMsg(parentId, addParentUpdate.request.parentId);

	if (!addParentClient.call(addParentUpdate)) {
		LOG(ERROR) << "Failed to call service Add Parent";
		return false;
	}

	return true;
}

bool SceneGraphROSCommunicator::removeParent(Id id, Id parentId) {
	LOG(DEBUG) << "SceneGraphROSCommunicator: removing parent Node";

//	removeParentUpdate.request.id = id;
//	removeParentUpdate.request.parentId = parentId;
	SceneGraphTypeCasts::convertIdToRosMsg(id, removeParentUpdate.request.id);
	SceneGraphTypeCasts::convertIdToRosMsg(parentId, removeParentUpdate.request.parentId);

	if (!removeParentClient.call(removeParentUpdate)) {
		LOG(ERROR) << "Failed to call service Remove Parent";
		return false;
	}

	return true;
}

//bool SceneGraphROSCommunicator::resentSceneGraphCallback(brics_3d_msgs::ResentSceneGraph::Request& request, brics_3d_msgs::ResentSceneGraph::Response& response) {
//	SceneGraphToUpdatesTraverser
//}


//void SceneGraphROSCommunicator::addIdToSubGraphBlacklist(Id subGraphId) {
//	subGraphBlackList.push_back(subGraphId);
//}
//
//void SceneGraphROSCommunicator::setEnableBlackList(bool enableBlackList) {
//	this->enableBlackList = enableBlackList;
//}
//
//bool SceneGraphROSCommunicator::getEnableBlackList() {
//	return enableBlackList;
//}
//
//bool isInABlacklistedSubGraph() {
//	return false;
//}

}

}

/* EOF */
