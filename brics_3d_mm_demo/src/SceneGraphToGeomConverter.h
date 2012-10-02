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
#include <brics_3d/core/ITriangleMesh.h>
#include <brics_3d/worldModel/sceneGraph/Node.h>
#include <brics_3d/worldModel/sceneGraph/Group.h>
#include <brics_3d/worldModel/sceneGraph/Transform.h>
#include <brics_3d/worldModel/sceneGraph/GeometricNode.h>
#include <brics_3d/worldModel/sceneGraph/INodeVisitor.h>
#include <brics_3d/worldModel/sceneGraph/SceneGraphFacade.h>

/* BRICS_MM includes */
#include <brics_mm/core/environment/geom/GeomContainer.h>
#include <brics_mm/core/environment/geom/TriangleSet.h>

namespace brics_mm {

/**
 * Traverses RSG and generates appropriate brics_mm::Geoms.
 */
class SceneGraphToGeomConverter : public brics_3d::rsg::INodeVisitor {
public:
	SceneGraphToGeomConverter(brics_3d::rsg::SceneGraphFacade* facadeHandle, unsigned int referenceNodeId, brics_mm::GeomContainer* geoms);
	virtual ~SceneGraphToGeomConverter();

	virtual void visit(brics_3d::rsg::Node* node);
	virtual void visit(brics_3d::rsg::Group* node);
	virtual void visit(brics_3d::rsg::Transform* node);
	virtual void visit(brics_3d::rsg::GeometricNode* node);

	/**
	 * @param[out] geoms Pointer to a GeomContainer that will be filled with data whil traversing the graph.
	 */
	virtual void reset(brics_mm::GeomContainer* geoms);

	/**
	 * @brief Overridable "template method" (for example for an ID aware version) for for resolving the transfrom between the reference frame and the fame valid in the currently visited GeometricNode.
	 * @param node The currently visited GeometricNode.
	 */
	virtual brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr doGetTransformFromReferenceToGeom(brics_3d::rsg::GeometricNode* node);

	static void convertTriangleMesh(brics_3d::ITriangleMesh* mesh, brics_mm::TriangleSet* geomMesh, double geometryScaleFactor = 1.0);
	static void convertTransform(brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, brics_mm::Transform* geomTransform, double geometryScaleFactor = 1.0);


protected:

	brics_3d::rsg::SceneGraphFacade* facadeHandle;
	unsigned int referenceNodeId;

	brics_mm::GeomContainer* geoms;

	double geometryScaleFactor;

};

}

#endif /* SCENEGRAPHTOGEOMCONVERTER_H_ */

/* EOF */
