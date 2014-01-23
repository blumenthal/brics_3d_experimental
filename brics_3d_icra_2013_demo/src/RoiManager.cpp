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


#include "RoiManager.h"

using namespace brics_3d::rsg;

namespace brics_3d {

RoiManager::RoiManager(brics_3d::WorldModel* wmHandle) :
		IFunctionBlock(wmHandle) {
	roiBoxSizeX = 2.0; //[m]
	roiBoxSizeY = 2.0; //[m]
	roiBoxSizeZ = 2.0; //[m]

	roiCenterX = 0;
	roiCenterY = 0.2;
	roiCenterZ = 1.1;
	roiPitch = 0.2 * M_PI_2;
	roiYaw = 0;
}

RoiManager::~RoiManager() {
	
}

void RoiManager::configure(brics_3d::ParameterSet parameters) {
	double parameterValued = 0.0;

	/* ROI dimensions */
	if (parameters.hasDouble("roiBoxSizeX", parameterValued)) {
		roiBoxSizeX = parameterValued;
		LOG(INFO) << "Setting parameter roiBoxSizeX to " << parameterValued;
	}
	if (parameters.hasDouble("roiBoxSizeY", parameterValued)) {
		roiBoxSizeY = parameterValued;
		LOG(INFO) << "Setting parameter roiBoxSizeY to " << parameterValued;
	}
	if (parameters.hasDouble("roiBoxSizeZ", parameterValued)) {
		roiBoxSizeZ = parameterValued;
		LOG(INFO) << "Setting parameter roiBoxSizeZ to " << parameterValued;
	}

	if (parameters.hasDouble("roiCenterX", parameterValued)) {
		roiCenterX = parameterValued;
		LOG(INFO) << "Setting parameter roiCenterX to " << parameterValued;
	}
	if (parameters.hasDouble("roiCenterY", parameterValued)) {
		roiCenterY = parameterValued;
		LOG(INFO) << "Setting parameter roiCenterY to " << parameterValued;
	}
	if (parameters.hasDouble("roiCenterZ", parameterValued)) {
		roiCenterZ = parameterValued;
		LOG(INFO) << "Setting parameter roiCenterZ to " << parameterValued;
	}
	if (parameters.hasDouble("roiPitch", parameterValued)) {
		roiPitch = parameterValued;
		LOG(INFO) << "Setting parameter roiPitch to " << parameterValued;
	}
}

void RoiManager::setData(std::vector<brics_3d::rsg::Id>& inputDataIds) {
	this->inputDataIds = inputDataIds; //make a copy
}

void RoiManager::execute() {
	LOG(INFO) << "RoiManager: Adding a new ROI.";
	if (inputDataIds.size() < 1) {
		LOG(WARNING) << "RoiManager: Not enough input IDs.";
		return;
	}

	brics_3d::rsg::Id rootId = inputDataIds[0];

	Eigen::AngleAxis<double> rotation(roiPitch, Eigen::Vector3d(1,0,0));
	Transform3d transformation;
	transformation = Eigen::Affine3d::Identity();
	transformation.translate(Eigen::Vector3d(roiCenterX,roiCenterY,roiCenterZ));
	transformation.rotate(rotation);
	brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44(&transformation));

	brics_3d::rsg::Id tfBox1Id = 0;
	brics_3d::rsg::Id Box1Id = 0;
	vector<brics_3d::rsg::Attribute> tmpAttributes;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","roi_box_tf"));
	wm->scene.addTransformNode(rootId, tfBox1Id, tmpAttributes, transform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));


	brics_3d::rsg::Box::BoxPtr box1(new brics_3d::rsg::Box(roiBoxSizeX, roiBoxSizeY, roiBoxSizeZ));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","roi_box"));
//	tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	tmpAttributes.push_back(Attribute("rsgInfo","non_shared"));
	wm->scene.addGeometricNode(tfBox1Id, Box1Id, tmpAttributes, box1, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
	LOG(DEBUG) << "ROI Box added with ID " << Box1Id;

	/* Here comes a basic robot skeleton */
	brics_3d::rsg::Id tfWorldToRobotId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","world_to_robot_tf"));
	brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr initialWorldToRobotTransform(new brics_3d::HomogeneousMatrix44());
	wm->scene.addTransformNode(wm->scene.getRootId(), tfWorldToRobotId, tmpAttributes, initialWorldToRobotTransform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

	brics_3d::rsg::Id tfRobotToSensorId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","robot_to_sensor_tf"));
	brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr initialRobotToSensorTransform(new brics_3d::HomogeneousMatrix44());
	LOG(DEBUG) << "current times stamp for frame = "<< timer.getCurrentTime();
	wm->scene.addTransformNode(tfWorldToRobotId, tfRobotToSensorId, tmpAttributes, initialRobotToSensorTransform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));



	/* We will add a hook for the processing data */
	brics_3d::rsg::Id sensorGroupId;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","sensor"));
	wm->scene.addGroup(tfRobotToSensorId, sensorGroupId, tmpAttributes);
	LOG(DEBUG) << "Sensor group added with ID " << sensorGroupId;

	/* We will add a hook for the scene objects */
	brics_3d::rsg::Id sceneObjectsGroupId;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","sceneObjects"));
	wm->scene.addGroup(wm->scene.getRootId(), sceneObjectsGroupId, tmpAttributes);
	LOG(DEBUG) << "Scene objects group  added with ID " << sensorGroupId;

}

void RoiManager::getData(std::vector<brics_3d::rsg::Id>& newDataIds) {
	newDataIds = this->outputDataIds;
}

}

/* EOF */
