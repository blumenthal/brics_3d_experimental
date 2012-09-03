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

#ifndef SCENEGRAPHTYPECASTS_H_
#define SCENEGRAPHTYPECASTS_H_

/* BRICS_3D includes */
#include <worldModel/sceneGraph/SceneGraphFacade.h>

#include <worldModel/sceneGraph/Box.h>
#include <worldModel/sceneGraph/Cylinder.h>
#include <core/HomogeneousMatrix44.h>
#include <core/Logger.h>

/* BRICS_3D <-> ROS types */
#include "brics_3d_msgs/GetRootId.h"

#include "brics_3d_msgs/GetNodes.h"
#include "brics_3d_msgs/GetNodeAttributes.h"
#include "brics_3d_msgs/GetNodeParents.h"
#include "brics_3d_msgs/GetGroupChildren.h"
#include "brics_3d_msgs/GetTransform.h"
#include "brics_3d_msgs/GetGeometry.h"
#include "brics_3d_msgs/GetTransformForNode.h"

#include "brics_3d_msgs/AddNode.h"
#include "brics_3d_msgs/AddGroup.h"
#include "brics_3d_msgs/AddTransformNode.h"
#include "brics_3d_msgs/AddGeometricNode.h"
#include "brics_3d_msgs/SetNodeAttributes.h"
#include "brics_3d_msgs/SetTransform.h"
#include "brics_3d_msgs/DeleteNode.h"
#include "brics_3d_msgs/AddParent.h"

#include <tf/tf.h>

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Helper class to cast between BRICS_3D and ROS mesage types
 */
class SceneGraphTypeCasts {
public:
	SceneGraphTypeCasts(){};
	virtual ~SceneGraphTypeCasts(){};

	inline static void convertAttributesToRosMsg(vector<Attribute>& attributes, std::vector< brics_3d_msgs::Attribute >&  convertedAttributes) {
		convertedAttributes.resize(attributes.size());
		for (unsigned int i = 0; i < static_cast<unsigned int>(attributes.size()); ++i) {
			convertedAttributes[i].key = attributes[i].key;
			convertedAttributes[i].value = attributes[i].value;
		}
	}

	inline static void convertRosMsgToAttributes(std::vector< brics_3d_msgs::Attribute >&  attributes, vector<Attribute>& convertedAttributes) {
		convertedAttributes.clear();
		for (unsigned int i = 0; i < static_cast<unsigned int>(attributes.size()); ++i) {
			convertedAttributes.push_back(Attribute(attributes[i].key ,attributes[i].value));
		}
	}

	inline static void convertIdsToRosMsg(vector<unsigned int>& ids, std::vector< uint32_t >&  convertedIds) {
		convertedIds.resize(ids.size());
		for (unsigned int i = 0; i < static_cast<unsigned int>(ids.size()); ++i) {
			convertedIds[i] = ids[i];
		}
	}

	inline static void convertRosMsgToIds(std::vector< uint32_t >&  ids, vector<unsigned int>& convertedIds) {
		convertedIds.resize(ids.size());
		for (unsigned int i = 0; i < static_cast<unsigned int>(ids.size()); ++i) {
			convertedIds[i] = ids[i];
		}
	}

	inline static void convertTimeStampToRosMsg(BRICS_3D::RSG::TimeStamp& timeStamp, ros::Time& convertedTimeStamp) {
		//not needed and not possible;
	}

	inline static void convertRosMsgToTimeStamp(ros::Time& timeStamp, BRICS_3D::RSG::TimeStamp& convertedTimeStamp) { //here we _have_ to create a new timestamp
//		if (convertedTimeStamp) {
//			delete convertedTimeStamp;
//			convertedTimeStamp = 0;
//		}
		BRICS_3D::RSG::TimeStamp tmpTimeStamp(timeStamp.toSec()*1000.0);
//		LOG(DEBUG) << "convertRosMsgToTimeStamp: " << timeStamp << " -> " << timeStamp.toSec()*1000.0;
		convertedTimeStamp += tmpTimeStamp; //assumes default constructor;
	}

	inline static void convertTransformToRosMsg(BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform,  geometry_msgs::TransformStamped& convertedTransform){

		tf::Transform tmpTransform;
		convertHomogeniousMatrixToTfTransform(transform, tmpTransform);
//		converteTransform.header.stamp = ros::Time::now();
//		converteTransform.header.frame_id = rootFrameId;
//		objectSceneFrameID.str("");
//		objectSceneFrameID << "scene_object_" << resultObjects[i].id;
//		converteTransform.child_frame_id = objectSceneFrameID.str();
		convertedTransform.transform.translation.x = tmpTransform.getOrigin().getX();
		convertedTransform.transform.translation.y = tmpTransform.getOrigin().getY();
		convertedTransform.transform.translation.z = tmpTransform.getOrigin().getZ();
		convertedTransform.transform.rotation.x = tmpTransform.getRotation().getX();
		convertedTransform.transform.rotation.y = tmpTransform.getRotation().getY();
		convertedTransform.transform.rotation.z = tmpTransform.getRotation().getZ();
		convertedTransform.transform.rotation.w = tmpTransform.getRotation().getW();
	}

	inline static void convertRosMsgToTransform(geometry_msgs::TransformStamped transform, BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& convertedTransform){
		tf::StampedTransform tmpTransform;
		tf::transformStampedMsgToTF(transform, tmpTransform);
		convertTfTransformToHomogeniousMatrix(tmpTransform, convertedTransform);
	}

	inline static void convertShapeToRosMsg(BRICS_3D::RSG::Shape::ShapePtr shape, arm_navigation_msgs::Shape& convertedShape){
		LOG(DEBUG) << "convertShapeToRosMsg: ";
		//		RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pointCloud(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
//		pointCloud = boost::dynamic_pointer_cast<PointCloud<BRICS_3D::PointCloud3D> >(shape);
//		RSG::Mesh<BRICS_3D::ITriangleMesh>::MeshPtr mesh(new RSG::Mesh<BRICS_3D::ITriangleMesh>());
//		mesh = boost::dynamic_pointer_cast<RSG::Mesh<BRICS_3D::ITriangleMesh> >(shape);
		RSG::Box::BoxPtr box(new RSG::Box());
		box =  boost::dynamic_pointer_cast<RSG::Box>(shape);
		RSG::Cylinder::CylinderPtr cylinder(new RSG::Cylinder());
		cylinder =  boost::dynamic_pointer_cast<RSG::Cylinder>(shape);

		if (box !=0) {
			LOG(DEBUG) << "                 -> Found a new box.";
			convertedShape.type = arm_navigation_msgs::Shape::BOX;
			convertedShape.dimensions.resize(3);
			convertedShape.dimensions[0] = box->getSizeX();
			convertedShape.dimensions[1] = box->getSizeY();
			convertedShape.dimensions[2] = box->getSizeZ();
		} else if (cylinder !=0) {
			LOG(DEBUG) << "                 -> Found a new cylinder.";
			convertedShape.type = arm_navigation_msgs::Shape::CYLINDER;
			convertedShape.dimensions.resize(2);
			convertedShape.dimensions[0] = cylinder->getRadius();
			convertedShape.dimensions[1] = cylinder->getHeight();
		} else {
			LOG(ERROR) << "convertShapeToRosMsg: Shape type not yet supported.";
		}
	}

	inline static void convertRosMsgToShape(arm_navigation_msgs::Shape shape, BRICS_3D::RSG::Shape::ShapePtr& convertedShape) {
		LOG(DEBUG) << "convertRosMsgToShape: ";
		switch(shape.type) {
		case arm_navigation_msgs::Shape::BOX: {
			LOG(DEBUG) << "                 -> Found a new box.";
			BRICS_3D::RSG::Box::BoxPtr newBox(new BRICS_3D::RSG::Box());
			newBox->setSizeX(shape.dimensions[0]);
			newBox->setSizeY(shape.dimensions[1]);
			newBox->setSizeZ(shape.dimensions[2]);
			convertedShape = boost::dynamic_pointer_cast<RSG::Shape>(newBox); // = newBox;

//			RSG::Box::BoxPtr box(new RSG::Box());
//			box =  boost::dynamic_pointer_cast<RSG::Box>(convertedShape);
//			assert(box!=0);

		} break;
		case arm_navigation_msgs::Shape::CYLINDER: {
			LOG(DEBUG) << "                 -> Found a new cylinder.";
			BRICS_3D::RSG::Cylinder::CylinderPtr newCylinder(new BRICS_3D::RSG::Cylinder());
			newCylinder->setRadius(shape.dimensions[0]);
			newCylinder->setHeight(shape.dimensions[1]);
			convertedShape = newCylinder;

		} break;
		case arm_navigation_msgs::Shape::SPHERE:
			LOG(ERROR) << "convertRosMsgToShape: Sphere type not yet supported.";
			break;
		case arm_navigation_msgs::Shape::MESH:
			LOG(ERROR) << "convertRosMsgToShape: Mesh type not yet supported.";
			break;
		}
	}

	/* Some helper functions */
	inline static void convertTfTransformToHomogeniousMatrix (const tf::Transform& tfTransform, BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix)
	{
		double mv[12];

		tfTransform.getBasis().getOpenGLSubMatrix(mv);
		tf::Vector3 origin = tfTransform.getOrigin();

		double* matrixPtr = transformMatrix->setRawData();

		/* matrices are column-major */
		matrixPtr[0] = mv[0]; matrixPtr[4] = mv[4]; matrixPtr[8] = mv[8];   matrixPtr[12] = origin.x();
		matrixPtr[1] = mv[1]; matrixPtr[5] = mv[5]; matrixPtr[9] = mv[9];   matrixPtr[13] = origin.y();
		matrixPtr[2] = mv[2]; matrixPtr[6] = mv[6]; matrixPtr[10] = mv[10]; matrixPtr[14] = origin.z();
		matrixPtr[3] = 0;     matrixPtr[7] = 0;     matrixPtr[11] = 0;      matrixPtr[15] = 1;

	}

	inline static void convertHomogeniousMatrixToTfTransform (const BRICS_3D::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix, tf::Transform& tfTransform) {
		const double* matrixPtr = transformMatrix->getRawData();

		btVector3 translation;
		btMatrix3x3 rotation;

		translation.setX(matrixPtr[12]);
		translation.setY(matrixPtr[13]);
		translation.setZ(matrixPtr[14]);

		rotation.setValue(
				matrixPtr[0], matrixPtr[4], matrixPtr[8],
				matrixPtr[1], matrixPtr[5], matrixPtr[9],
				matrixPtr[2], matrixPtr[6], matrixPtr[10]
		);

		tfTransform.setOrigin(translation);
		tfTransform.setBasis(rotation);
	}


};

}

}

#endif /* SCENEGRAPHTYPECASTS_H_ */

/* EOF */
