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

#ifndef ROIADAPTER_H_
#define ROIADAPTER_H_

#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/util/Timer.h> // An abstract timer could also go into the WM
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>

using brics_3d::Logger;

namespace brics_3d {


class RoiAdapter : public rsg::IFunctionBlock {
public:
	RoiAdapter(brics_3d::WorldModel* wmHandle);
	virtual ~RoiAdapter();

	void configure(brics_3d::ParameterSet parameters);
	void setData(std::vector<unsigned int>& inputDataIds); 	//Input: ID0: bounding box by perception result  ID1: wich box to addjust
	void execute();
	void getData(std::vector<unsigned int>& newDataIds); 	//Output: the box id that forms the ROI

	/* data conventions */
	const static unsigned int existingRoiBoxIdInputIndex = 0;
	const static unsigned int percievedObjectBoundingBoxIdInputIndex = 1;

	const static unsigned int updatedRoiBoxIdOutputIndex = 0;

private:

	/* Utils */
	brics_3d::Timer timer;

	double maxX;
	double maxY;
	double maxZ;
};

}

#endif /* ROIADAPTER_H_ */

/* EOF */
