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

#include <string>

#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>

#include "brics_3d_msgs/GetNodes.h"
#include "brics_3d_msgs/SetTransform.h"

namespace brics_3d {

const static unsigned int INVALID_ID = 0;

struct SceneGraphTransformNodes {
public:
	SceneGraphTransformNodes() : id(INVALID_ID){}
	unsigned int id;
	std::string name;
	std::string tfParent;
};

class TfToSceneGraphConverter {
public:
	TfToSceneGraphConverter(ros::NodeHandle n) :
			node(n) {
			initialize();
		}

	virtual ~TfToSceneGraphConverter(){};

	void initialize () {

		std::stringstream serviceName;
		std::string serviceNameSpace = "/worldModel/";

		serviceName.str("");
		serviceName << serviceNameSpace << "getNodes";
		getNodesClient = node.serviceClient<brics_3d_msgs::GetNodes>(serviceName.str());

		serviceName.str("");
		serviceName << serviceNameSpace << "setTransform";
	    setTransformClient = node.serviceClient<brics_3d_msgs::SetTransform>(serviceName.str());

		maxTFCacheDuration = ros::Duration(1.0); //[s]

//		 /openni_depth_optical_frame
//	      frame_id: /base_footprint
//	    child_frame_id: /openni_camera
		SceneGraphTransformNodes tmpSceneGraphTransformSpec;
		tmpSceneGraphTransformSpec.id = INVALID_ID; // We will have to find out later.
		tmpSceneGraphTransformSpec.name = "robot_to_sensor_tf";
		tmpSceneGraphTransformSpec.tfParent = "/base_footprint";
		std::string tmpFrameId = "/openni_depth_optical_frame";
		tfToSceneGraphMapping.insert(std::make_pair(tmpFrameId, tmpSceneGraphTransformSpec));
	};

	void processTfTopic () {
		std::map <std::string, brics_3d::SceneGraphTransformNodes>::iterator iter = tfToSceneGraphMapping.begin();
		for (iter = tfToSceneGraphMapping.begin(); iter != tfToSceneGraphMapping.end(); iter++) {

			std::string tfFrameId = iter->first;
			std::string tfFrameReferenceId = iter->second.tfParent;
			tf::StampedTransform transform;
			try{
				tfListener.lookupTransform(tfFrameReferenceId, tfFrameId, ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
				ROS_WARN("%s",ex.what());
				continue;
			}

			if ( (ros::Time::now() - transform.stamp_) > maxTFCacheDuration ) { //simply ignore outdated TF frames
				ROS_WARN("TF found for %s. But it is outdated. Skipping it.", iter->first.c_str());
				continue;
			}
			ROS_INFO("TF found for %s.", iter->first.c_str());

			/*
			 * update scene graph
			 */

			/* resolve id if necessary */
			if (iter->second.id == INVALID_ID) {

				getNodesQuery.request.attributes.resize(1);
				getNodesQuery.request.attributes[0].key = "name";
				getNodesQuery.request.attributes[0].value = iter->second.name;

				if (!getNodesClient.call(getNodesQuery)) {
					ROS_ERROR("Failed to call service Get Node");
					continue;
				}

				ROS_INFO("Found %i IDs for node in the scene graph with name %s", getNodesQuery.response.ids.size(), iter->second.name.c_str());
				for (unsigned int i = 0; i < getNodesQuery.response.ids.size(); ++i) {
					ROS_INFO("	ID = %i", getNodesQuery.response.ids[i]);
				}

				/* we attempt to take the first one (arbitrary choice) */
				if (getNodesQuery.response.ids.size() > 0) {
					iter->second.id = getNodesQuery.response.ids[0];
				} else {
					ROS_WARN("Cannot find a node in the scene graph with name %s", iter->second.name.c_str());
					continue;
				}
			}

			/* do the update */

			ROS_INFO("updating transform");
			setTransformUpdate.request.id = iter->second.id;
			setTransformUpdate.request.stamp = transform.stamp_;
			tf::transformStampedTFToMsg(transform, setTransformUpdate.request.transform);
			std::cout << setTransformUpdate.request.transform;

	//		setTransformUpdate.request.transform = tf::StampedTransform transform; // ::geometry_msgs::TransformStamped

			if (!setTransformClient.call(setTransformUpdate)) {
				ROS_ERROR("Failed to call service Set Transform");
				continue;
			}

		}
	}

private:

	/// The ROS node handle
	ros::NodeHandle node;

	/// Receives TF
	tf::TransformListener tfListener;

	ros::Duration maxTFCacheDuration;

	/// Mapping
	std::map <std::string, brics_3d::SceneGraphTransformNodes> tfToSceneGraphMapping;

	ros::ServiceClient getNodesClient;
	brics_3d_msgs::GetNodes getNodesQuery;

	ros::ServiceClient setTransformClient;
	brics_3d_msgs::SetTransform setTransformUpdate;

};

}  // namespace brics_3d


int main(int argc, char **argv) {

	ros::init(argc, argv, "tf_to_scene_graph_node");
	ros::NodeHandle n;
	brics_3d::TfToSceneGraphConverter tfToSceneGrapgh(n);


	ros::Rate rate(1); // (in Hz)
	while (n.ok()){
		ros::spinOnce();
		tfToSceneGrapgh.processTfTopic();
		rate.sleep();
	}

	return 0;
}

/* EOF */
