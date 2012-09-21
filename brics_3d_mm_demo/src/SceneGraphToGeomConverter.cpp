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

#include "SceneGraphToGeomConverter.h"
#include <core/HomogeneousMatrix44.h>
#include <core/PointCloud3D.h>
#include <worldModel/sceneGraph/PointCloud.h>
#include <worldModel/sceneGraph/Mesh.h>
#include <worldModel/sceneGraph/Box.h>
#include <worldModel/sceneGraph/Cylinder.h>
//#include "core/Logger.h"
#include "brics_mm/utils/Logger.h"

using BRICS_MM::Logger;

namespace BRICS_MM {



SceneGraphToGeomConverter::SceneGraphToGeomConverter(BRICS_3D::RSG::SceneGraphFacade* facadeHandle, unsigned int referenceNodeId, BRICS_MM::GeomContainer* geoms) : INodeVisitor(downwards)  {
	assert(facadeHandle != 0);
	this->facadeHandle = facadeHandle;
	this->referenceNodeId = referenceNodeId;
	geometryScaleFactor = 1000; //1.0;
	reset(geoms);
}

SceneGraphToGeomConverter::~SceneGraphToGeomConverter() {

}

void SceneGraphToGeomConverter::reset(BRICS_MM::GeomContainer* geoms) {
	assert (geoms != 0);
	this->geoms = geoms; //clear geoms or append? For now append.
}

void SceneGraphToGeomConverter::visit(BRICS_3D::RSG::Node* node){
	/* do nothing */
}

void SceneGraphToGeomConverter::visit(BRICS_3D::RSG::Group* node){
	/* do nothing */
}

void SceneGraphToGeomConverter::visit(BRICS_3D::RSG::Transform* node){
	/* do nothing */
}

void SceneGraphToGeomConverter::visit(BRICS_3D::RSG::GeometricNode* node){
	assert (node != 0);
	BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToGeom = doGetTransformFromReferenceToGeom(node);
	BRICS_MM::Transform transformToGeom;

	convertTransform(transformFromReferenceToGeom, &transformToGeom, geometryScaleFactor);
	std::stringstream nodeID;
	std::stringstream nodeName;
	nodeID.str("");
	nodeID << node->getId();
	nodeName.str("");
//	nodeName << node->getAttributes();

	BRICS_3D::RSG::Shape::ShapePtr shape = node->getShape();
	BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pointCloud(new BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pointCloud = boost::dynamic_pointer_cast<BRICS_3D::RSG::PointCloud<BRICS_3D::PointCloud3D> >(shape);
	BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh>::MeshPtr mesh(new BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh>());
	mesh = boost::dynamic_pointer_cast<BRICS_3D::RSG::Mesh<BRICS_3D::ITriangleMesh> >(shape);
	BRICS_3D::RSG::Box::BoxPtr box(new BRICS_3D::RSG::Box());
	box =  boost::dynamic_pointer_cast<BRICS_3D::RSG::Box>(shape);
	BRICS_3D::RSG::Cylinder::CylinderPtr cylinder(new BRICS_3D::RSG::Cylinder());
	cylinder =  boost::dynamic_pointer_cast<BRICS_3D::RSG::Cylinder>(shape);

	LOG(DEBUG) << "SceneGraphToGeomConverter: adding geode";

	if (pointCloud !=0) {
		LOG(DEBUG) << "               -> Ignoring point cloud.";

	} else if (mesh !=0) {
		LOG(DEBUG) << "                 -> Adding a new mesh.";

		BRICS_MM::TriangleSet* geomMesh = new BRICS_MM::TriangleSet();
		convertTriangleMesh(mesh->data.get(), geomMesh, geometryScaleFactor);
		geomMesh->applyTransform(transformToGeom);
		geomMesh->setId(nodeID.str());
		//name?
		geoms->addGeom(geomMesh);

	} else if (box !=0) {
		LOG(DEBUG) << "                 -> Adding a new box.";

		BRICS_MM::TriangleSet* geomBox = new BRICS_MM::TriangleSet();
		*geomBox = BRICS_MM::TriangleSet::makeBox(
				box->getSizeX() * geometryScaleFactor,
				box->getSizeY() * geometryScaleFactor,
				box->getSizeZ() * geometryScaleFactor);
		geomBox->applyTransform(transformToGeom);
		geomBox->setId(nodeID.str());
		geoms->addGeom(geomBox);

	} else if (cylinder !=0) {
		LOG(DEBUG) << "                 -> Adding a new cylinder.";

		BRICS_MM::TriangleSet* geomCylinder = new BRICS_MM::TriangleSet();
		*geomCylinder = BRICS_MM::TriangleSet::makeCylinder(
				cylinder->getRadius() * geometryScaleFactor,
				cylinder->getHeight() * geometryScaleFactor);
		geomCylinder->applyTransform(transformToGeom);
		geomCylinder->setId(nodeID.str());
		geoms->addGeom(geomCylinder);

	} else {
		LOG(WARNING) << "               -> Unsupported geometry type. Cannot add this.";

	}

}

BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr SceneGraphToGeomConverter::doGetTransformFromReferenceToGeom(BRICS_3D::RSG::GeometricNode* node) {
	assert (node != 0);
	BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToGeom;

	if(!facadeHandle->getTransformForNode(node->getId(), referenceNodeId, BRICS_3D::RSG::TimeStamp(), transformFromReferenceToGeom)) {
		LOG(ERROR) << "SceneGraphToGeomConverter: Cannot find appropriate transfrom. Returning identity.";

		BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new BRICS_3D::HomogeneousMatrix44());
		transformFromReferenceToGeom = identity;
	}

	return transformFromReferenceToGeom;


}

void SceneGraphToGeomConverter::convertTriangleMesh(BRICS_3D::ITriangleMesh* mesh, BRICS_MM::TriangleSet* geomMesh, double geometryScaleFactor) {
	assert (mesh != 0);
	assert (geomMesh !=0 );
	BRICS_3D::Point3D tmpVertex;

	int indexCount = 0;
	for (int i = 0; i < mesh->getSize(); ++i) { // loop over all triangles
		for (int j = 0; j <= 2; ++j) { // loop over the three vertices per triangle
			tmpVertex = *mesh->getTriangleVertex(i,j);
			tmpVertex = tmpVertex * geometryScaleFactor; //Some scaling
			geomMesh->addVertex(
					tmpVertex.getX(),
					tmpVertex.getY(),
					tmpVertex.getZ());

			indexCount++;
		}
		geomMesh->addTriangle(indexCount-3, indexCount-2, indexCount-1);
		//LOG(DEBUG) << "adding tiangle with indices" << indexCount-2 << ", " << indexCount-1 << ", " << indexCount;
	}

	assert(static_cast<unsigned int>(mesh->getSize()) == geomMesh->getNumTriangles());
}

void SceneGraphToGeomConverter::convertTransform(BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, BRICS_MM::Transform* geomTransform, double geometryScaleFactor) {
	assert(transform != 0);
	assert(geomTransform != 0);

	const double *matrix = transform->getRawData(); // column-row order!
	geomTransform->trans_[BRICS_MM::Transform::R00] = matrix[0];
	geomTransform->trans_[BRICS_MM::Transform::R01] = matrix[4];
	geomTransform->trans_[BRICS_MM::Transform::R02] = matrix[8];

	geomTransform->trans_[BRICS_MM::Transform::R10] = matrix[1];
	geomTransform->trans_[BRICS_MM::Transform::R11] = matrix[5];
	geomTransform->trans_[BRICS_MM::Transform::R12] = matrix[9];

	geomTransform->trans_[BRICS_MM::Transform::R21] = matrix[2];
	geomTransform->trans_[BRICS_MM::Transform::R22] = matrix[6];
	geomTransform->trans_[BRICS_MM::Transform::R22] = matrix[10];

	geomTransform->trans_[BRICS_MM::Transform::TX] = matrix[12] * geometryScaleFactor;
	geomTransform->trans_[BRICS_MM::Transform::TY] = matrix[13] * geometryScaleFactor;
	geomTransform->trans_[BRICS_MM::Transform::TZ] = matrix[14] * geometryScaleFactor;

}

}

/* EOF */
