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
#ifndef RSG_SCENEGRAPHROSBROADCASTER_H_
#define RSG_SCENEGRAPHROSBROADCASTER_H_

#include "brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "brics_3d/worldModel/sceneGraph/PointCloud.h"
#include "brics_3d/worldModel/sceneGraph/Mesh.h"
#include "brics_3d/worldModel/sceneGraph/Box.h"
#include "brics_3d/worldModel/sceneGraph/Cylinder.h"
#include "brics_3d/worldModel/sceneGraph/Attribute.h"

/* ROS includes */
#include "ros/ros.h"

#include "brics_3d_msgs/SceneGraphUpdate.h"

namespace brics_3d {

namespace rsg {

class SceneGraphROSBroadcaster : public brics_3d::rsg::ISceneGraphUpdateObserver {
public:
	SceneGraphROSBroadcaster(ros::NodeHandle n);
	virtual ~SceneGraphROSBroadcaster();

	/* implemetntations of observer interface */
	bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes);
	bool setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
	bool deleteNode(unsigned int id);
	bool addParent(unsigned int id, unsigned int parentId);
	bool removeParent(unsigned int id, unsigned int parentId);


protected:

	void initialize();
	void doSendMessage(brics_3d_msgs::SceneGraphUpdate& update);

	ros::NodeHandle node;

	ros::Publisher sceneGraphUpdatesPublisher;

};

}

}

#endif /* RSG_SCENEGRAPHROSBROADCASTER_H_ */

/* EOF */
