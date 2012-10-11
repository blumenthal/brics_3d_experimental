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
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/worldModel/sceneGraph/PointCloud.h>
#include <brics_3d/worldModel/sceneGraph/Mesh.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/Cylinder.h>
//#include "core/Logger.h"
#include <brics_mm/utils/Logger.h>

using brics_mm::Logger;

namespace brics_mm {



SceneGraphToGeomConverter::SceneGraphToGeomConverter(brics_3d::rsg::SceneGraphFacade* facadeHandle, unsigned int referenceNodeId, brics_mm::GeomContainer* geoms) : INodeVisitor(downwards)  {
	assert(facadeHandle != 0);
	this->facadeHandle = facadeHandle;
	this->referenceNodeId = referenceNodeId;
	geometryScaleFactor = 1000; //1.0;
	reset(geoms);
}

SceneGraphToGeomConverter::~SceneGraphToGeomConverter() {

}

void SceneGraphToGeomConverter::reset(brics_mm::GeomContainer* geoms) {
	assert (geoms != 0);
	this->geoms = geoms; //clear geoms or append? For now append.
}

void SceneGraphToGeomConverter::visit(brics_3d::rsg::Node* node){
	/* do nothing */
}

void SceneGraphToGeomConverter::visit(brics_3d::rsg::Group* node){
	/* do nothing */
}

void SceneGraphToGeomConverter::visit(brics_3d::rsg::Transform* node){
	/* do nothing */
}

void SceneGraphToGeomConverter::visit(brics_3d::rsg::GeometricNode* node){
	assert (node != 0);
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToGeom = doGetTransformFromReferenceToGeom(node);
	brics_mm::Transform transformToGeom;

	convertTransform(transformFromReferenceToGeom, &transformToGeom, geometryScaleFactor);
	std::stringstream nodeID;
	std::stringstream nodeName;
	nodeID.str("");
	nodeID << node->getId();
	nodeName.str("");
//	nodeName << node->getAttributes();

	brics_3d::rsg::Shape::ShapePtr shape = node->getShape();
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pointCloud(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	pointCloud = boost::dynamic_pointer_cast<brics_3d::rsg::PointCloud<brics_3d::PointCloud3D> >(shape);
	brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr mesh(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
	mesh = boost::dynamic_pointer_cast<brics_3d::rsg::Mesh<brics_3d::ITriangleMesh> >(shape);
	brics_3d::rsg::Box::BoxPtr box(new brics_3d::rsg::Box());
	box =  boost::dynamic_pointer_cast<brics_3d::rsg::Box>(shape);
	brics_3d::rsg::Cylinder::CylinderPtr cylinder(new brics_3d::rsg::Cylinder());
	cylinder =  boost::dynamic_pointer_cast<brics_3d::rsg::Cylinder>(shape);

	LOG(DEBUG) << "SceneGraphToGeomConverter: adding geode";

	if (pointCloud !=0) {
		LOG(DEBUG) << "               -> Ignoring point cloud.";

	} else if (mesh !=0) {
		LOG(DEBUG) << "                 -> Adding a new mesh.";

		brics_mm::TriangleSet* geomMesh = new brics_mm::TriangleSet();
		convertTriangleMesh(mesh->data.get(), geomMesh, geometryScaleFactor);
		geomMesh->applyTransform(transformToGeom);
		geomMesh->setId(nodeID.str());
		//name?
		geoms->addGeom(geomMesh);

	} else if (box !=0) {
		LOG(DEBUG) << "                 -> Adding a new box.";

		brics_mm::TriangleSet* geomBox = new brics_mm::TriangleSet();
		*geomBox = brics_mm::TriangleSet::makeBox(
				box->getSizeX() * geometryScaleFactor,
				box->getSizeY() * geometryScaleFactor,
				box->getSizeZ() * geometryScaleFactor);
		geomBox->applyTransform(transformToGeom);
		geomBox->setId(nodeID.str());
		geoms->addGeom(geomBox);

	} else if (cylinder !=0) {
		LOG(DEBUG) << "                 -> Adding a new cylinder.";

		brics_mm::TriangleSet* geomCylinder = new brics_mm::TriangleSet();
		*geomCylinder = brics_mm::TriangleSet::makeCylinder(
				cylinder->getRadius() * geometryScaleFactor,
				cylinder->getHeight() * geometryScaleFactor);
		geomCylinder->applyTransform(transformToGeom);
		geomCylinder->setId(nodeID.str());
		geoms->addGeom(geomCylinder);

	} else {
		LOG(WARNING) << "               -> Unsupported geometry type. Cannot add this.";

	}

}

brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr SceneGraphToGeomConverter::doGetTransformFromReferenceToGeom(brics_3d::rsg::GeometricNode* node) {
	assert (node != 0);
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToGeom;

	if(!facadeHandle->getTransformForNode(node->getId(), referenceNodeId, brics_3d::rsg::TimeStamp(), transformFromReferenceToGeom)) {
		LOG(ERROR) << "SceneGraphToGeomConverter: Cannot find appropriate transfrom. Returning identity.";

		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new brics_3d::HomogeneousMatrix44());
		transformFromReferenceToGeom = identity;
	}

	return transformFromReferenceToGeom;


}

void SceneGraphToGeomConverter::convertTriangleMesh(brics_3d::ITriangleMesh* mesh, brics_mm::TriangleSet* geomMesh, double geometryScaleFactor) {
	assert (mesh != 0);
	assert (geomMesh !=0 );
	brics_3d::Point3D tmpVertex;

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

void SceneGraphToGeomConverter::convertTransform(brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, brics_mm::Transform* geomTransform, double geometryScaleFactor) {
	assert(transform != 0);
	assert(geomTransform != 0);

	/*
	 * column-row layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */
	const double *matrix = transform->getRawData(); // column-row order!
	geomTransform->trans_[brics_mm::Transform::R00] = matrix[0];
	geomTransform->trans_[brics_mm::Transform::R01] = matrix[4];
	geomTransform->trans_[brics_mm::Transform::R02] = matrix[8];

	geomTransform->trans_[brics_mm::Transform::R10] = matrix[1];
	geomTransform->trans_[brics_mm::Transform::R11] = matrix[5];
	geomTransform->trans_[brics_mm::Transform::R12] = matrix[9];

	geomTransform->trans_[brics_mm::Transform::R20] = matrix[2];
	geomTransform->trans_[brics_mm::Transform::R21] = matrix[6];
	geomTransform->trans_[brics_mm::Transform::R22] = matrix[10];

	geomTransform->trans_[brics_mm::Transform::TX] = matrix[12] * geometryScaleFactor;
	geomTransform->trans_[brics_mm::Transform::TY] = matrix[13] * geometryScaleFactor;
	geomTransform->trans_[brics_mm::Transform::TZ] = matrix[14] * geometryScaleFactor;

//	LOG(DEBUG) << "brics_3d transform: " << std::endl << *transform;
//	LOG(DEBUG) << "brics_mm geom transform: " << std::endl << *geomTransform;
//	LOG(DEBUG) << "with scale: " << geometryScaleFactor;

}

}

/* EOF */
