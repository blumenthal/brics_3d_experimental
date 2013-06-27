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


#ifndef SCENEASSOCIATION_H_
#define SCENEASSOCIATION_H_

#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/OutdatedDataIdAwareDeleter.h>
#include <brics_3d/util/Timer.h>
#include <brics_3d/util/Benchmark.h>

namespace brics_3d {


class SceneAssociation : public rsg::IFunctionBlock {
public:
	SceneAssociation(brics_3d::WorldModel* wmHandle);
	virtual ~SceneAssociation();

	void configure(brics_3d::ParameterSet parameters);
	void execute();

	/* data conventions */
	const static unsigned int percievedObjectsSubGraphRootIndex = 0;
	const static unsigned int sceneObjectsSubGraphRootIndex = 1;

private:

	brics_3d::Timer timer;
	brics_3d::Benchmark* associationBenchmark;
	brics_3d::Benchmark* associationDistances;

	/// The tag used for the scene objects
	brics_3d::rsg::Attribute sceneObjectTag;
	brics_3d::rsg::Attribute percievedSceneObjectTag;

	double associationDistanceTreshold;

	/// This one Will delete outdated scene objects
	brics_3d::rsg::OutdatedDataIdAwareDeleter* objectDeleter;
};


}

#endif /* SCENEASSOCIATION_H_ */

/* EOF */
