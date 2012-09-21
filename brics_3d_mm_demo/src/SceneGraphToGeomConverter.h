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

#ifndef SCENEGRAPHTOGEOMCONVERTER_H_
#define SCENEGRAPHTOGEOMCONVERTER_H_

/* BRICS_3D includes */
#include <core/ITriangleMesh.h>
#include <worldModel/sceneGraph/Node.h>
#include <worldModel/sceneGraph/Group.h>
#include <worldModel/sceneGraph/Transform.h>
#include <worldModel/sceneGraph/GeometricNode.h>
#include <worldModel/sceneGraph/INodeVisitor.h>
#include <worldModel/sceneGraph/SceneGraphFacade.h>

/* BRICS_MM includes */
#include "brics_mm/core/environment/geom/GeomContainer.h"
#include "brics_mm/core/environment/geom/TriangleSet.h"

namespace BRICS_MM {

/**
 * Traverses RSG and generates appropriate BRICS_MM::Geoms.
 */
class SceneGraphToGeomConverter : public BRICS_3D::RSG::INodeVisitor {
public:
	SceneGraphToGeomConverter(BRICS_3D::RSG::SceneGraphFacade* facadeHandle, unsigned int referenceNodeId, BRICS_MM::GeomContainer* geoms);
	virtual ~SceneGraphToGeomConverter();

	virtual void visit(BRICS_3D::RSG::Node* node);
	virtual void visit(BRICS_3D::RSG::Group* node);
	virtual void visit(BRICS_3D::RSG::Transform* node);
	virtual void visit(BRICS_3D::RSG::GeometricNode* node);

	/**
	 * @param[out] geoms Pointer to a GeomContainer that will be filled with data whil traversing the graph.
	 */
	virtual void reset(BRICS_MM::GeomContainer* geoms);

	/**
	 * @brief Overridable "template method" (for example for an ID aware version) for for resolving the transfrom between the reference frame and the fame valid in the currently visited GeometricNode.
	 * @param node The currently visited GeometricNode.
	 */
	virtual BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr doGetTransformFromReferenceToGeom(BRICS_3D::RSG::GeometricNode* node);

	static void convertTriangleMesh(BRICS_3D::ITriangleMesh* mesh, BRICS_MM::TriangleSet* geomMesh, double geometryScaleFactor = 1.0);
	static void convertTransform(BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, BRICS_MM::Transform* geomTransform, double geometryScaleFactor = 1.0);


protected:

	BRICS_3D::RSG::SceneGraphFacade* facadeHandle;
	unsigned int referenceNodeId;

	BRICS_MM::GeomContainer* geoms;

	double geometryScaleFactor;

};

}

#endif /* SCENEGRAPHTOGEOMCONVERTER_H_ */

/* EOF */
