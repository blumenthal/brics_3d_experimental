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

#include "RoiAdapter.h"

using namespace brics_3d::rsg;

namespace brics_3d {


RoiAdapter::RoiAdapter(brics_3d::WorldModel* wmHandle) :
		IFunctionBlock(wmHandle) {
	maxX = 4; //[m]
	maxY = 4;
	maxZ = 4;
}

RoiAdapter::~RoiAdapter() {

}

void RoiAdapter::configure(brics_3d::ParameterSet parameters) {

}

void RoiAdapter::setData(std::vector<brics_3d::rsg::Id>& inputDataIds) {
	this->inputDataIds = inputDataIds; //make a copy
}

void RoiAdapter::execute() {
	LOG(INFO) << "RoiAdapter: Adding a new ROI.";
	if (inputDataIds.size() < 1) {
		LOG(WARNING) << "RoiAdapter: Not enough input IDs. Skipping.";
		return;
	}

	double newX = 0;
	double newY = 0;
	double newZ = 0;
	double scaleFactor = 4;//3;//4;//2;//10;	/* make dimensions a bit more relax (4 is good) */
	double increment = 0.1;
	brics_3d::rsg::Id existingRoiBoxId = inputDataIds[existingRoiBoxIdInputIndex];
	brics_3d::rsg::Id tfBox1Id = 0;
	vector<brics_3d::rsg::Attribute> tmpAttributes;

	std::vector<brics_3d::rsg::Id> parents;
	wm->scene.getNodeParents(existingRoiBoxId, parents);
	assert(parents.size() == 1);
	tfBox1Id = parents[0];

	if (inputDataIds.size() < 2) {
		LOG(INFO) << "Empty percievedObjectBoundingBox. Relaxing ROI.";

		for (unsigned int i = 0; i < inputDataIds.size(); ++i) {
			LOG(INFO) << "inputDataIds[" << i << "] = " << inputDataIds[i];
		}

		/* get box data for existing box  */
		Shape::ShapePtr resultShape;
		TimeStamp resultTime;
		wm->scene.getGeometry(existingRoiBoxId, resultShape, resultTime);
		brics_3d::rsg::Box::BoxPtr resultBox;
		resultBox = boost::dynamic_pointer_cast<brics_3d::rsg::Box>(resultShape);
		assert(resultBox != 0);

		if (resultBox->getSizeX() < maxX) {
			newX = resultBox->getSizeX() + increment;
		}
		if (resultBox->getSizeY() < maxY) {
			newY = resultBox->getSizeY() + increment;
		}
		if (resultBox->getSizeZ() < maxZ) {
			newZ = resultBox->getSizeZ() + increment;
		}

		/* (we do not update the center here) */

	} else {

		brics_3d::rsg::Id percievedObjectBoundingBoxId = inputDataIds[percievedObjectBoundingBoxIdInputIndex];

		for (unsigned int i = 0; i < inputDataIds.size(); ++i) {
			LOG(INFO) << "inputDataIds[" << i << "] = " << inputDataIds[i];
		}

		/* get box data for percieved box  */
		Shape::ShapePtr resultShape;
		TimeStamp resultTime;
		wm->scene.getGeometry(percievedObjectBoundingBoxId, resultShape, resultTime);
		brics_3d::rsg::Box::BoxPtr resultBox;
		resultBox = boost::dynamic_pointer_cast<brics_3d::rsg::Box>(resultShape);
		assert(resultBox != 0);

		newX = resultBox->getSizeX() * scaleFactor;
		newY = resultBox->getSizeY() * scaleFactor;
		newZ = resultBox->getSizeZ() * scaleFactor;


		/* adjust center of existing box */
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr updatedRoiTransform;
		wm->scene.getTransformForNode(percievedObjectBoundingBoxId, wm->scene.getRootId(), brics_3d::rsg::TimeStamp(timer.getCurrentTime()), updatedRoiTransform); // percievedObjectBoundingBox expressed in: parent frame of existingRoiBoxId's parent tf (==root)
		LOG(INFO) << "updatedRoiTransform " << *updatedRoiTransform;
		wm->scene.setTransform(tfBox1Id, updatedRoiTransform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

	}

	/* adjust dimensions of box */
	brics_3d::rsg::Box::BoxPtr newBox(new brics_3d::rsg::Box(newX, newY, newZ));
	wm->scene.getNodeAttributes(existingRoiBoxId, tmpAttributes);

	brics_3d::rsg::Id newBoxId = 0;
	wm->scene.addGeometricNode(tfBox1Id, newBoxId, tmpAttributes, newBox, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
	LOG(DEBUG) << "Adjusted ROI Box added with ID " << newBoxId << " to parent with ID " << tfBox1Id;
	outputDataIds.clear();
	outputDataIds.push_back(newBoxId); // result id

	/* delete old box */
	wm->scene.deleteNode(existingRoiBoxId);

}

void RoiAdapter::getData(std::vector<brics_3d::rsg::Id>& newDataIds) {
	newDataIds = this->outputDataIds;
}

}

/* EOF */
