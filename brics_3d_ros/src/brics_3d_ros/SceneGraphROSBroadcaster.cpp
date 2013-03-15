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

#include "SceneGraphROSBroadcaster.h"
#include "SceneGraphTypeCasts.h"
#include <brics_3d/core/Logger.h>

using brics_3d::Logger;

namespace brics_3d {

namespace rsg {

SceneGraphROSBroadcaster::SceneGraphROSBroadcaster(ros::NodeHandle n) {
	this->node = n;
	initialize();
}

SceneGraphROSBroadcaster::~SceneGraphROSBroadcaster() {
	// TODO Auto-generated destructor stub
}

void SceneGraphROSBroadcaster::initialize() {
	sceneGraphUpdatesPublisher = node.advertise<brics_3d_msgs::SceneGraphUpdate>("scene_graph_updates", 10);
}


bool SceneGraphROSBroadcaster::addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::addNode";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::ADD_NODE;
	message.parentId = parentId;
	message.forcedId = forcedId;
	if (forcedId) {
		message.assignedId = assignedId;
	}
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, message.attributes);

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::addGroup";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::ADD_GROUP;
	message.parentId = parentId;
	message.forcedId = forcedId;
	if (forcedId) {
		message.assignedId = assignedId;
	}
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, message.attributes);

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::addTransformNode";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::ADD_TRANSFORM_NODE;
	message.parentId = parentId;
	message.forcedId = forcedId;
	if (forcedId) {
		message.assignedId = assignedId;
	}
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, message.attributes);
	SceneGraphTypeCasts::convertTransformToRosMsg(transform, message.transform);
	SceneGraphTypeCasts::convertTimeStampToRosMsg(timeStamp, message.stamp);

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::addGeometricNode";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::ADD_GEOMETRIC_NODE;
	message.parentId = parentId;
	message.forcedId = forcedId;
	if (forcedId) {
		message.assignedId = assignedId;
	}
	SceneGraphTypeCasts::convertAttributesToRosMsg(attributes, message.attributes);
	SceneGraphTypeCasts::convertShapeToRosMsg(shape, message.shape);
	SceneGraphTypeCasts::convertTimeStampToRosMsg(timeStamp, message.stamp);

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::setNodeAttributes";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::SET_NOTE_ATTRIBUTES;
	message.id = id;

	SceneGraphTypeCasts::convertAttributesToRosMsg(newAttributes, message.attributes);

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::setTransform";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::SET_TRANSFORM;
	message.id = id;

	SceneGraphTypeCasts::convertTransformToRosMsg(transform, message.transform);
	SceneGraphTypeCasts::convertTimeStampToRosMsg(timeStamp, message.stamp);

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::deleteNode(unsigned int id) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::deleteNode";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::DELETE_NOTE;
	message.id = id;

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::addParent(unsigned int id, unsigned int parentId) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::addParent";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::ADD_PARENT;
	message.id = id;
	message.parentId = parentId;

	doSendMessage(message);
	return true;
}

bool SceneGraphROSBroadcaster::removeParent(unsigned int id, unsigned int parentId) {
	LOG(DEBUG) << "SceneGraphROSBroadcaster::removeParent";
	brics_3d_msgs::SceneGraphUpdate message;

	message.command = brics_3d_msgs::SceneGraphUpdate::REMOVE_PARENT;
	message.id = id;
	message.parentId = parentId;

	doSendMessage(message);
	return true;
}


void SceneGraphROSBroadcaster::doSendMessage(brics_3d_msgs::SceneGraphUpdate& update) {
	sceneGraphUpdatesPublisher.publish(update);
}


}

}

/* EOF */
