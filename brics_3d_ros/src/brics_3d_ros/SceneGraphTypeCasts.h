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
#include <brics_3d/worldModel/sceneGraph/SceneGraphFacade.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/Cylinder.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/Logger.h>

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

#include "brics_3d_msgs/SceneObject.h"
#include "brics_3d_msgs/SceneObjects.h"

#include <tf/tf.h>

namespace brics_3d {

namespace rsg {

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

	inline static void convertRosMsgToAttributes(const std::vector< brics_3d_msgs::Attribute >&  attributes, vector<Attribute>& convertedAttributes) {
		convertedAttributes.clear();
		for (unsigned int i = 0; i < static_cast<unsigned int>(attributes.size()); ++i) {
			convertedAttributes.push_back(Attribute(attributes[i].key ,attributes[i].value));
		}
	}

	inline static void convertIdToRosMsg(brics_3d::rsg::Id id,  uint32_t& convertedId) {
		convertedId = brics_3d::rsg::uuidToUnsignedInt(id);
	}

	inline static void convertRosMsgToId(uint32_t id,  brics_3d::rsg::Id& convertedId) {
		convertedId = id;
	}

	inline static void convertIdsToRosMsg(vector<brics_3d::rsg::Id>& ids, std::vector< uint32_t >&  convertedIds) {
		convertedIds.resize(ids.size());
		for (unsigned int i = 0; i < static_cast<unsigned int>(ids.size()); ++i) {
//			convertedIds[i] = ids[i];
			convertIdToRosMsg(ids[i], convertedIds[i]);
		}
	}

	inline static void convertRosMsgToIds(const std::vector< uint32_t >&  ids, vector<brics_3d::rsg::Id>& convertedIds) {
		convertedIds.resize(ids.size());
		for (unsigned int i = 0; i < static_cast<unsigned int>(ids.size()); ++i) {
//			convertedIds[i] = ids[i];
			convertRosMsgToId(ids[i], convertedIds[i]);
		}
	}

	inline static void convertTimeStampToRosMsg(brics_3d::rsg::TimeStamp& timeStamp, ros::Time& convertedTimeStamp) {
		//not needed and not possible;
	}

	inline static void convertRosMsgToTimeStamp(const ros::Time& timeStamp, brics_3d::rsg::TimeStamp& convertedTimeStamp) { //here we _have_ to create a new timestamp
//		if (convertedTimeStamp) {
//			delete convertedTimeStamp;
//			convertedTimeStamp = 0;
//		}
		brics_3d::rsg::TimeStamp tmpTimeStamp(timeStamp.toSec()*1000.0);
//		LOG(DEBUG) << "convertRosMsgToTimeStamp: " << timeStamp << " -> " << timeStamp.toSec()*1000.0;
		convertedTimeStamp += tmpTimeStamp; //assumes default constructor;
	}

	inline static void convertTransformToRosMsg(brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform,  geometry_msgs::TransformStamped& convertedTransform){

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

	inline static void convertRosMsgToTransform(const geometry_msgs::TransformStamped transform, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& convertedTransform){
		tf::StampedTransform tmpTransform;
		tf::transformStampedMsgToTF(transform, tmpTransform);
		convertTfTransformToHomogeniousMatrix(tmpTransform, convertedTransform);
	}

	inline static bool convertShapeToRosMsg(brics_3d::rsg::Shape::ShapePtr shape, brics_3d_msgs::Shape& convertedShape){
		LOG(DEBUG) << "convertShapeToRosMsg: ";
		//		rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pointCloud(new rsg::PointCloud<brics_3d::PointCloud3D>());
//		pointCloud = boost::dynamic_pointer_cast<PointCloud<brics_3d::PointCloud3D> >(shape);
//		rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr mesh(new rsg::Mesh<brics_3d::ITriangleMesh>());
//		mesh = boost::dynamic_pointer_cast<rsg::Mesh<brics_3d::ITriangleMesh> >(shape);
		rsg::Box::BoxPtr box(new rsg::Box());
		box =  boost::dynamic_pointer_cast<rsg::Box>(shape);
		rsg::Cylinder::CylinderPtr cylinder(new rsg::Cylinder());
		cylinder =  boost::dynamic_pointer_cast<rsg::Cylinder>(shape);

		if (box !=0) {
			LOG(DEBUG) << "                 -> Found a new box.";
			convertedShape.type = brics_3d_msgs::Shape::BOX;
			convertedShape.dimensions.resize(3);
			convertedShape.dimensions[0] = box->getSizeX();
			convertedShape.dimensions[1] = box->getSizeY();
			convertedShape.dimensions[2] = box->getSizeZ();
		} else if (cylinder !=0) {
			LOG(DEBUG) << "                 -> Found a new cylinder.";
			convertedShape.type = brics_3d_msgs::Shape::CYLINDER;
			convertedShape.dimensions.resize(2);
			convertedShape.dimensions[0] = cylinder->getRadius();
			convertedShape.dimensions[1] = cylinder->getHeight();
		} else {
			LOG(ERROR) << "convertShapeToRosMsg: Shape type not yet supported.";
			return false;
		}
		return true;
	}

	inline static bool convertRosMsgToShape(brics_3d_msgs::Shape shape, brics_3d::rsg::Shape::ShapePtr& convertedShape) {
		LOG(DEBUG) << "convertRosMsgToShape: ";
		switch(shape.type) {
		case brics_3d_msgs::Shape::BOX: {
			LOG(DEBUG) << "                 -> Found a new box.";
			brics_3d::rsg::Box::BoxPtr newBox(new brics_3d::rsg::Box());
			newBox->setSizeX(shape.dimensions[0]);
			newBox->setSizeY(shape.dimensions[1]);
			newBox->setSizeZ(shape.dimensions[2]);
			convertedShape = boost::dynamic_pointer_cast<rsg::Shape>(newBox); // = newBox;

		} break;
		case brics_3d_msgs::Shape::CYLINDER: {
			LOG(DEBUG) << "                 -> Found a new cylinder.";
			brics_3d::rsg::Cylinder::CylinderPtr newCylinder(new brics_3d::rsg::Cylinder());
			newCylinder->setRadius(shape.dimensions[0]);
			newCylinder->setHeight(shape.dimensions[1]);
			convertedShape = newCylinder;

		} break;
		case brics_3d_msgs::Shape::SPHERE:
			LOG(ERROR) << "convertRosMsgToShape: Sphere type not yet supported.";
			break;
		case brics_3d_msgs::Shape::MESH:
			LOG(ERROR) << "convertRosMsgToShape: Mesh type not yet supported.";
			return false;
		}
		return true;
	}

	/* Some helper functions */
	inline static void convertTfTransformToHomogeniousMatrix (const tf::Transform& tfTransform, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix)
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

	inline static void convertHomogeniousMatrixToTfTransform (const brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix, tf::Transform& tfTransform) {
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

	inline static bool convertSceneObjectToRosMsg(brics_3d::SceneObject sceneObject, brics_3d_msgs::SceneObject& convertedSceneObject, std::string originFrameId) {
//		convertedSceneObject.id = sceneObject.id;
//		convertedSceneObject.parentId = sceneObject.parentId;
		convertIdToRosMsg(sceneObject.id, convertedSceneObject.id);
		convertIdToRosMsg(sceneObject.parentId, convertedSceneObject.parentId);
		convertAttributesToRosMsg(sceneObject.attributes, convertedSceneObject.attributes);

		std::stringstream objectSceneFrameID;
		convertTransformToRosMsg(sceneObject.transform, convertedSceneObject.transform);
		convertedSceneObject.transform.header.stamp = ros::Time::now();
		convertedSceneObject.transform.header.frame_id = originFrameId;
		objectSceneFrameID.str("");
		objectSceneFrameID << "scene_object_" << sceneObject.id;
		convertedSceneObject.transform.child_frame_id = objectSceneFrameID.str();

		if(!convertShapeToRosMsg(sceneObject.shape, convertedSceneObject.shape)) {
			return false;
		}

		return true;
	}

	inline static bool convertSceneObjectsToRosMsg(vector<brics_3d::SceneObject> sceneObjects, brics_3d_msgs::SceneObjects& convertedSceneObjects, std::string originFrameId) {
		convertedSceneObjects.sceneObjects.resize(sceneObjects.size());
		for (unsigned int i = 0; i < sceneObjects.size(); ++i) {
			if(!convertSceneObjectToRosMsg(sceneObjects[i], convertedSceneObjects.sceneObjects[i], originFrameId)) {
				//LOG(WARNING) << "convertSceneObjectsToRosMsg: An error occured while converting scene object " << i;
				//return false;
			}
		}
		return true;
	}

	inline static bool convertPoseMsgToHomogeniousMatrix(geometry_msgs::PoseStamped pose, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix) {
		tf::Transform tfTransform;
		btVector3 translation;
		btQuaternion rotation;

		translation.setX(pose.pose.position.x);
		translation.setY(pose.pose.position.y);
		translation.setZ(pose.pose.position.z);
		rotation.setX(pose.pose.orientation.x);
		rotation.setY(pose.pose.orientation.y);
		rotation.setZ(pose.pose.orientation.z);
		rotation.setW(pose.pose.orientation.w);

		tfTransform.setOrigin(translation);
		tfTransform.setRotation(rotation);
		convertTfTransformToHomogeniousMatrix(tfTransform, transformMatrix);

		return true;
	}
};

}

}

#endif /* SCENEGRAPHTYPECASTS_H_ */

/* EOF */
