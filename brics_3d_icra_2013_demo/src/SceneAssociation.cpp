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


#include "SceneAssociation.h"
#include "brics_3d/worldModel/sceneGraph/AttributeFinder.h"
#include "brics_3d/worldModel/sceneGraph/Cylinder.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/Logger.h"

using brics_3d::Logger;
using namespace brics_3d::rsg;

#include <limits>
#include <math.h>

namespace brics_3d {

SceneAssociation::SceneAssociation(brics_3d::WorldModel* wmHandle) : IFunctionBlock(wmHandle) {

	//Default values:
	sceneObjectTag = brics_3d::rsg::Attribute("taskType","sceneObject");
	percievedSceneObjectTag = brics_3d::rsg::Attribute("taskType","percievedSceneObject");
	associationDistanceTreshold = 0.07;//0.5;//0.1;//03;// [m]
	objectDeleter = new rsg::OutdatedDataIdAwareDeleter(&wmHandle->scene);

//	associationBenchmark = new brics_3d::Benchmark("scene_association");
//	associationBenchmark->output << "# xDistance yDistance zDistance  EucledianDistance Threshold Association(1=true;0=false)" << std::endl;
//
//	associationDistances = new brics_3d::Benchmark("scene_association_distances");
//	associationDistances->output << "# xDistance yDistance zDistance  EucledianDistance";
}

SceneAssociation::~SceneAssociation() {
	delete objectDeleter;
}

void SceneAssociation::configure(brics_3d::ParameterSet parameters) {
	//TODO configure sceneObjectTag
}

void SceneAssociation::execute() {
	/*
	 *               root
	 *                 |
	 *        sensor-- +------scene
	 *           |              |
	 *     obj---+--obj   scObj-+-scObj---scObj
	 *    attr1    attr2  attr1   attr1   attr2
	 *      ^        ^
	 *      |        |
	 *
	 *     update existing or insert new objects into scene, based on Eukledion distance of cnetoid (and attributes)
	 *
	 *     NOTE/Assumption: _All_ objects have the sceneObjectsTag and potentially more.
	 */

	LOG(INFO) << "SceneAssociation: Trying to associate.";

	unsigned int sensorRootId;
	unsigned int sceneRootId;
	assert(inputDataIds.size()>=2);
	sensorRootId = inputDataIds[percievedObjectsSubGraphRootIndex];
	sceneRootId = inputDataIds[sceneObjectsSubGraphRootIndex];

	vector<brics_3d::rsg::Attribute> queryAttributes;
	queryAttributes.push_back(sceneObjectTag); 

	/* A) Find percieved scene objects */
	brics_3d::rsg::AttributeFinder attributeFinder;
	attributeFinder.setQueryAttributes(queryAttributes);
	vector<unsigned int> percivedResultIds;
	wm->scene.executeGraphTraverser(&attributeFinder, sensorRootId);
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeFinder.getMatchingNodes().size()) ; ++i) {
		percivedResultIds.push_back((*attributeFinder.getMatchingNodes()[i]).getId());
	}
	LOG(DEBUG) << "Number of found percieved scene objects: " << percivedResultIds.size();

	/* B) Find existing scene objects */
	vector<unsigned int> sceneObjectsResultIds;
	attributeFinder.reset();
	queryAttributes.clear();
	queryAttributes.push_back(sceneObjectTag); 
	attributeFinder.setQueryAttributes(queryAttributes);
	wm->scene.executeGraphTraverser(&attributeFinder, sceneRootId);
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeFinder.getMatchingNodes().size()) ; ++i) {
		sceneObjectsResultIds.push_back((*attributeFinder.getMatchingNodes()[i]).getId());
	}
	LOG(DEBUG) << "Number of found existing scene objects: " << sceneObjectsResultIds.size();

	/* C) Associate each by each by smallest Eucledian distace */
	for (unsigned int i = 0; i < static_cast<unsigned int>(percivedResultIds.size()) ; ++i) {

		unsigned int index = -1;
		double minSquardDistanceToExistingObjects = std::numeric_limits<double>::max();
		const double* matrixPtr;

		/* Assumption percived object _is a_ transform node; the rest will be ignored. */
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr percievedTransform;
		if(!wm->scene.getTransform(percivedResultIds[i], TimeStamp(timer.getCurrentTime()), percievedTransform)) {
			LOG(WARNING) << "SceneAssociation: Percieved node with ID "<< percivedResultIds[i] <<" is not a transform node. Ignoring it.";
			continue;
		}
		LOG(DEBUG) << "SceneAssociation: Trying to associate percieved node with ID " << percivedResultIds[i];

		double squardDistanceToExistingObjects;
		for (unsigned int j = 0; j < static_cast<unsigned int>(sceneObjectsResultIds.size()) ; ++j) {

			/* Assumption scene object _is a_ transform node; the rest will be ignored. */
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr sceneObjectTransform;
			if(!wm->scene.getTransform(sceneObjectsResultIds[j], TimeStamp(timer.getCurrentTime()), percievedTransform)) {
				LOG(WARNING) << "SceneAssociation: Scene object node with ID "<< sceneObjectsResultIds[j] <<" is not a transform node. Ignoring it.";
				continue;
			}
			LOG(DEBUG) << "SceneAssociation: 	... scene object node with ID " << sceneObjectsResultIds[j];

			/*DBG the other TF */
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformBetweenObjectsDBG;
			wm->scene.getTransformForNode(sceneObjectsResultIds[j], percivedResultIds[i], TimeStamp(timer.getCurrentTime()), transformBetweenObjectsDBG);
			matrixPtr = transformBetweenObjectsDBG->getRawData();
			double xDistanceDBG = matrixPtr[12];
			double yDistanceDBG = matrixPtr[13];
			double zDistanceDBG = matrixPtr[14];
//			associationDistances->output << xDistanceDBG << "\t" << yDistanceDBG << "\t" << zDistanceDBG << "\t";


			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformBetweenObjects;
			wm->scene.getTransformForNode(percivedResultIds[i], sceneObjectsResultIds[j], TimeStamp(timer.getCurrentTime()), transformBetweenObjects);
			matrixPtr = transformBetweenObjects->getRawData();

			double xDistance = matrixPtr[12];
			double yDistance = matrixPtr[13];
			double zDistance = matrixPtr[14];
//			associationDistances->output << xDistance << "\t" << yDistance << "\t" << zDistance << "\t";

			squardDistanceToExistingObjects =(xDistance * xDistance) +
					(yDistance * yDistance) + (zDistance * zDistance);
			LOG(DEBUG) << "SceneAssociation: 	... they differ " << squardDistanceToExistingObjects;
//			associationDistances->output << sqrt(squardDistanceToExistingObjects) << std::endl;

			if (squardDistanceToExistingObjects < minSquardDistanceToExistingObjects) {
				minSquardDistanceToExistingObjects = squardDistanceToExistingObjects;
				index = j;
			}
		}


		LOG(DEBUG) << "SceneAssociation:  	... shortest distance " << minSquardDistanceToExistingObjects << " to found result object.";
		if ((sceneObjectsResultIds.size() > 0) && (index < sceneObjectsResultIds.size())) {
			assert (index < sceneObjectsResultIds.size());
			LOG(DEBUG)<< "SceneAssociation:  	... distance relates to object with ID" << sceneObjectsResultIds[index];
		} else {
			LOG(DEBUG)<< "SceneAssociation:  	... scene has been empty so far.";
		}

		if (minSquardDistanceToExistingObjects < (associationDistanceTreshold * associationDistanceTreshold) ) {
			assert (index < sceneObjectsResultIds.size());

			/* update existing */
			LOG(DEBUG) << "SceneAssociation: Updating existing scene object with object ID: " << sceneObjectsResultIds[index] << ". It fits to percieved object with ID: " <<  percivedResultIds[i];
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformSceneToPervievedObject;
			wm->scene.getTransformForNode(percivedResultIds[i], sceneRootId, TimeStamp(timer.getCurrentTime()), transformSceneToPervievedObject);
			wm->scene.setTransform(sceneObjectsResultIds[index], transformSceneToPervievedObject, TimeStamp(timer.getCurrentTime())); //Assumption scene is the parent of _this_ tarnadform

			/* We could actually also update the geometry if we like ...*/

		} else {

			/* insert */
			LOG(INFO) << "SceneAssociation: Inserting a new scene object";
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformSceneToPervievedObject;
			wm->scene.getTransformForNode(percivedResultIds[i], sceneRootId, TimeStamp(timer.getCurrentTime()), transformSceneToPervievedObject);
			unsigned int newSceneObjectId;
			vector<brics_3d::rsg::Attribute> tmpAttributes;
			wm->scene.getNodeAttributes(percivedResultIds[i], tmpAttributes); // we take over all attributes
			tmpAttributes.push_back(Attribute("debugInfo","scnObj"));
			wm->scene.addTransformNode(sceneRootId, newSceneObjectId, tmpAttributes, transformSceneToPervievedObject, TimeStamp(timer.getCurrentTime())); //Assumption scene is the parent of _this_ tarnadform

			/* we will take over all children */
			vector <unsigned int> children;
			wm->scene.getGroupChildren(percivedResultIds[i], children);
			for (unsigned int childIndex = 0; childIndex < children.size(); ++childIndex) {
				wm->scene.addParent(children[childIndex], newSceneObjectId);
			}


		}
	}
	wm->scene.executeGraphTraverser(objectDeleter, sceneRootId);
}


}

/* EOF */
