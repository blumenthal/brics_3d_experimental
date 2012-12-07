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

#include "SceneGraphROSListener.h"
#include "SceneGraphTypeCasts.h"
#include <brics_3d/core/Logger.h>

using brics_3d::Logger;

namespace brics_3d {

namespace rsg {

SceneGraphROSListener::SceneGraphROSListener(ros::NodeHandle n, brics_3d::WorldModel* wm) {
	this->node = n;
	this->wm = wm;
}

SceneGraphROSListener::~SceneGraphROSListener() {

}

void SceneGraphROSListener::initialize() {
	sceneGraphUpdatesSubscriber = node.subscribe("/scene_graph_update", 1000, &SceneGraphROSListener::handleSceneGraphUpdate, this);
}

void SceneGraphROSListener::handleSceneGraphUpdate(const brics_3d_msgs::SceneGraphUpdate& update) {

	switch (update.command) {
		case brics_3d_msgs::SceneGraphUpdate::ADD_NODE:
		    doAddNode(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::ADD_GROUP:
			doAddGroup(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::ADD_TRANSFORM_NODE:
			doAddTransformNode(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::ADD_GEOMATIC_NODE: //FIXME: typo
			doAddGeometricNode(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::SET_NOTE_ATTRIBUTES:
			doSetNodeAttributes(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::SET_TRANSFORM:
			doSetTransform(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::DELETE_NOTE:
			doDeleteNode(update);
			break;

		case brics_3d_msgs::SceneGraphUpdate::ADD_PARENT:
			doAddParent(update);
			break;

		default:
			LOG(ERROR) << "SceneGraphROSListener: undefined command type in incoming update message.";
			break;
	}
}

bool SceneGraphROSListener::doAddNode(const brics_3d_msgs::SceneGraphUpdate& update) {
	vector<brics_3d::rsg::Attribute> attributes;
	unsigned int assignedId;
	assignedId = update.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.

	SceneGraphTypeCasts::convertRosMsgToAttributes(update.attributes, attributes);
	return wm->scene.addNode(update.parentId, assignedId, attributes, update.forcedId);
}

bool SceneGraphROSListener::doAddGroup(const brics_3d_msgs::SceneGraphUpdate& update){
	unsigned int assignedId;
	vector<brics_3d::rsg::Attribute> attributes;
	assignedId = update.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.

	SceneGraphTypeCasts::convertRosMsgToAttributes(update.attributes, attributes);
	return wm->scene.addGroup(update.parentId, assignedId, attributes, update.forcedId);
}

bool SceneGraphROSListener::doAddTransformNode(const brics_3d_msgs::SceneGraphUpdate& update) {
	unsigned int assignedId;
	vector<brics_3d::rsg::Attribute> attributes;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
	TimeStamp timeStamp;
	assignedId = update.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.


	SceneGraphTypeCasts::convertRosMsgToAttributes(update.attributes, attributes);
	SceneGraphTypeCasts::convertRosMsgToTransform(update.transform, transform);
	SceneGraphTypeCasts::convertRosMsgToTimeStamp(update.stamp, timeStamp);
	return wm->scene.addTransformNode(update.parentId, assignedId, attributes, transform, timeStamp, update.forcedId);
}

bool SceneGraphROSListener::doAddGeometricNode(const brics_3d_msgs::SceneGraphUpdate& update) {
	unsigned int assignedId;
	vector<brics_3d::rsg::Attribute> attributes;
	Shape::ShapePtr shape;
	TimeStamp timeStamp;
	assignedId = update.assignedId; //This is an [in, out] parameter. Input ignored by default. Only iff forcedId is set to true than this value will be taken as input.

	LOG(DEBUG) << "WorldModelQueryServer: adding GeometricNode with parent ID: " << update.parentId;

	SceneGraphTypeCasts::convertRosMsgToAttributes(update.attributes, attributes);
	SceneGraphTypeCasts::convertRosMsgToShape(update.shape, shape);
	SceneGraphTypeCasts::convertRosMsgToTimeStamp(update.stamp, timeStamp);
	return wm->scene.addGeometricNode(update.parentId, assignedId, attributes, shape, timeStamp, update.forcedId);

}

bool SceneGraphROSListener::doSetNodeAttributes(const brics_3d_msgs::SceneGraphUpdate& update) {
	vector<brics_3d::rsg::Attribute> newAttributes;

	SceneGraphTypeCasts::convertRosMsgToAttributes(update.attributes, newAttributes);
	return wm->scene.setNodeAttributes(update.id, newAttributes);
}

bool SceneGraphROSListener::doSetTransform(const brics_3d_msgs::SceneGraphUpdate& update) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
	TimeStamp timeStamp;

	SceneGraphTypeCasts::convertRosMsgToTransform(update.transform, transform);
	SceneGraphTypeCasts::convertRosMsgToTimeStamp(update.stamp, timeStamp);
	return wm->scene.setTransform(update.id ,transform, timeStamp);
}

bool SceneGraphROSListener::doDeleteNode(const brics_3d_msgs::SceneGraphUpdate& update) {
	return wm->scene.deleteNode(update.id);
}

bool SceneGraphROSListener::doAddParent(const brics_3d_msgs::SceneGraphUpdate& update) {
	return wm->scene.addParent(update.id, update.parentId);
}

}

}

/* EOF */
