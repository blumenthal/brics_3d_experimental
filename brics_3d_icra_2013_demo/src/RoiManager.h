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


#ifndef ROIMANAGER_H_
#define ROIMANAGER_H_

#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/util/Timer.h> // An abstract timer could also go into the WM
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>

using brics_3d::Logger;

namespace brics_3d {

/**
 * Function block to setup a Box ROI
 */
class RoiManager : public rsg::IFunctionBlock {
public:
	RoiManager(brics_3d::WorldModel* wmHandle);
	virtual ~RoiManager();

	void configure(brics_3d::ParameterSet parameters);
	void setData(std::vector<unsigned int>& inputDataIds); //Input: where to hook
	void execute();
	void getData(std::vector<unsigned int>& newDataIds);

private:

	/* init the ROI Box */
	double roiBoxSizeX; //[m]
	double roiBoxSizeY; //[m]
	double roiBoxSizeZ; //[m]
	double roiCenterX;
	double roiCenterY;
	double roiCenterZ;
	double roiPitch;
	double roiYaw; //TODO

	/* Utils */
	brics_3d::Timer timer;
};

}

#endif /* ROIMANAGER_H_ */

/* EOF */
