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

#include <iostream>
#include <fstream>

#include "../brics_3d_ros/WorldModelQueryServer.h"
#include "../brics_3d_ros/SceneGraphROSCommunicator.h"
#include "../brics_3d_ros/SceneGraphResentServer.h"
#include "../brics_3d_ros/SceneGraphROSBroadcaster.h"

#include "brics_3d/worldModel/sceneGraph/DotVisualizer.h"
#include "brics_3d/worldModel/sceneGraph/DotGraphGenerator.h"
#include "brics_3d/worldModel/sceneGraph/SceneGraphFacade.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"

#ifdef ENABLE_OSG
	#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#endif

using brics_3d::Logger;

double rotationValue = 0;
brics_3d::Timer timer;

void rotateFrame(brics_3d::WorldModel* wm, unsigned int transformNodeId) {

	Eigen::Vector3d axis(1,1,1);
	axis.normalize();
	Eigen::AngleAxis<double> rotation(rotationValue, axis);
	//hen setting up an AngleAxis object, the axis vector must be normalized.
	Transform3d transformation;
	transformation = Eigen::Affine3d::Identity();
	transformation.translate(Eigen::Vector3d(0.15,0,0));
	transformation.rotate(rotation);
	brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44(&transformation));

	wm->scene.setTransform(transformNodeId, transform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

	rotationValue += 0.01;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "world_model_node");
	ros::NodeHandle n;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

#ifdef ENABLE_OSG
	// Visualization tool for world model
	brics_3d::rsg::OSGVisualizer* wmObserver = new brics_3d::rsg::OSGVisualizer();
	wm->scene.attachUpdateObserver(wmObserver); //enable visualization
#endif

	brics_3d::rsg::DotVisualizer dbgObserver(&wm->scene);
//	wm->scene.attachUpdateObserver(&dbgObserver);

	brics_3d::rsg::SceneGraphROSCommunicator updater(n, "/brics_mm/worldModel/"); // uses service calls
//	brics_3d::rsg::SceneGraphROSBroadcaster updater(n); // uses messages
	wm->scene.attachUpdateObserver(&updater);

	brics_3d::WorldModelQueryServer wmServer(n, wm);
	wmServer.setServiceNameSpace("/worldModel/");
	wmServer.initialize();

	brics_3d::rsg::SceneGraphResentServer resentServer(n, wm, &updater);
	resentServer.setServiceNameSpace("/worldModel/");
	resentServer.initialize();


	LOG(INFO) << "Ready.";

	bool testRotation = true;
	if(testRotation) {
		brics_3d::rsg::TimeStamp stamp(timer.getCurrentTime());

		unsigned int id1;
		vector<brics_3d::rsg::Attribute> attributes;
		attributes.push_back(brics_3d::rsg::Attribute("name","testTransform1"));
		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform1(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0.5, 0, 0));
		wm->scene.addTransformNode(wm->scene.getRootId(), id1, attributes, transform1, stamp);

		unsigned int id2;
		attributes.clear();
		attributes.push_back(brics_3d::rsg::Attribute("name","testTransform2"));
		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform2(new brics_3d::HomogeneousMatrix44());
		wm->scene.addTransformNode(id1, id2, attributes, transform2, stamp);

		wm->scene.addParent(id2, wm->scene.getRootId());

		ros::Rate rate(10); // (in Hz)
		while (n.ok()){
			ros::spinOnce();
			rotateFrame(wm, id1);
			rotateFrame(wm, id2);
			rate.sleep();
		}
	}

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
//	ros::spin();
	delete wm;
	return 0;
}



/* EOF */
