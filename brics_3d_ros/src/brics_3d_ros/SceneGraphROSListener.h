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

#ifndef RSG_SCENEGRAPHROSLISTENER_H_
#define RSG_SCENEGRAPHROSLISTENER_H_

/* ROS includes */
#include "ros/ros.h"

/* BRICS_3D includes */
#include <brics_3d/worldModel/WorldModel.h>

#include <brics_3d_msgs/SceneGraphUpdate.h>

namespace brics_3d {

namespace rsg {

/**
 * @brief A listener that maps ROS scene graph updetes to the respective function calls.
 */
class SceneGraphROSListener {
public:

	/**
	 * @brief Constructor with a ROS handle.
	 * @param n ROS handle
	 * @param wm Handle the the worldmodel. All ROS service calls will be forwarded to this one.
	 */
	SceneGraphROSListener(ros::NodeHandle n, brics_3d::WorldModel* wm);

	/**
	 * @brief Default destructor.
	 */
	virtual ~SceneGraphROSListener();

	/**
	 * @brief Subscribe to scene graph update topic.
	 */
	void initialize();

	/**
	 * @brief Handle an incoming update message for the scene graph.
	 * The appopriate update functionals of the world model handel wm will be called.
	 * @param update The recived update message for the scene graph.
	 */
	void handleSceneGraphUpdate(const brics_3d_msgs::SceneGraphUpdate& update);


protected:

	//Functions that do the actual work (template method)
    bool doAddNode(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doAddGroup(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doAddTransformNode(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doAddGeometricNode(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doSetNodeAttributes(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doSetTransform(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doDeleteNode(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doAddParent(const brics_3d_msgs::SceneGraphUpdate& update);
    bool doRemoveParent(const brics_3d_msgs::SceneGraphUpdate& update);


	/// The ROS node handle.
	ros::NodeHandle node;

	/// Handle to scene graph based world model.
	brics_3d::WorldModel* wm;

	/// Subsriber for recieving new updates.
	ros::Subscriber sceneGraphUpdatesSubscriber;

};

}

}

#endif /* RSG_SCENEGRAPHROSLISTENER_H_ */

/* EOF */
