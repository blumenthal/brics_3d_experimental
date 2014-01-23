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

/* ROS includes */
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/transform_broadcaster.h"

/* BRICS_3D includes */
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/Version.h>
#include <brics_3d/util/PCLTypecaster.h>
#include <brics_3d/util/Timer.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/PointCloud.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>


#ifdef ENABLE_OSG
	#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#endif

#include <brics_3d_ros/SceneGraphTypeCasts.h>
#include <brics_3d_ros/WorldModelQueryServer.h>
#include <brics_3d_ros/SceneGraphROSCommunicator.h>
#include <brics_3d_ros/SceneGraphResentServer.h>

#include "SimpleSceneAnalysis.h"
#include "RoiManager.h"
#include "RoiAdapter.h"
#include "SceneAssociation.h"

#include <brics_3d_msgs/SceneObjects.h>

using brics_3d::Logger;
using namespace brics_3d::rsg;

//typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointType;

class SceneAnalyzer {

public:

	SceneAnalyzer(brics_3d::WorldModel* wmHandle) {
		wm = wmHandle;
		//brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG/*INFO*/); // Set logger level
		brics_3d::Logger::setMinLoglevel(brics_3d::Logger::INFO); // Set logger level
		brics_3d::Logger::setLogfile("scene_analyzer_log.txt");
		LOG(INFO) << "Used version: " << brics_3d::Version::getVersionAsString();

		benchmark = new brics_3d::Benchmark("youbot_scene_analyzer_function_blocks");
		benchmark->output << "# Timings [ms] for: 1.) Conversion 2.) SimpleSceneAnalyzer 3.) dynamic ROI 4.) Association 6.) complete cycle time " << std::endl;


		count = 0;
		enableDynamicROI = true; //just a dafault
//		wm = new brics_3d::WorldModel();
#ifdef ENABLE_OSG
		wmObserver = new brics_3d::rsg::OSGVisualizer();
		wm->scene.attachUpdateObserver(wmObserver); //enable visualization
#endif
//		dbgObserver = new brics_3d::rsg::DotVisualizer(&wm->scene);
//		//dbgObserver->setKeepHistory(true); // If set to true _all_ graph snapshots will be saved as a file.
//		wm->scene.attachUpdateObserver(dbgObserver);

		lastPointCloudId = 0;
		sceneObjectTag = brics_3d::rsg::Attribute("taskType","sceneObject");

		roiBlock = new brics_3d::RoiManager(wm); // it is actually meant as an "inititial setup manager"
		perceptionBlock = new brics_3d::SimpleSceneAnalysis(wm);
		roiUpdateBlock = new brics_3d::RoiAdapter(wm);
		sceneAssociation = new brics_3d::SceneAssociation(wm);

	};
	~SceneAnalyzer() {
		delete perceptionBlock;
		delete roiBlock;
		delete roiUpdateBlock;
		delete sceneAssociation;
		delete dbgObserver;
		delete benchmark;
//		delete wm;
	};

	void callback (const pcl::PointCloud<PointType>::ConstPtr &cloud)
	{
		LOG(INFO) <<  "Receiving new point cloud";
		completeCycleTimer.reset();

		vector<brics_3d::rsg::Attribute> tmpAttributes;
		std::vector<brics_3d::rsg::Id> tmpResultIds;

		/*** RAW DATA ***/

		/* Create new brics_3d data structures */
		brics_3d::PointCloud3D::PointCloud3DPtr newPointCloud(new brics_3d::PointCloud3D());
		brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr newPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
		newPointCloudContainer->data=newPointCloud;

		timer.reset();
		converter.convertToBRICS3DDataType(cloud, newPointCloud.get());
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO) << "Point cloud has " << newPointCloud->getSize() << " points.";
		LOG(INFO) << "Timer: Conversion took " << timer.getElapsedTime() << "[ms]";

		/* Add new point cloud to world model */
		timer.reset();
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","sensor"));
		wm->scene.getNodes(tmpAttributes, tmpResultIds); // find node
		assert(tmpResultIds.size() == 1);
		brics_3d::rsg::Id sensorGroupId = tmpResultIds[0];
		brics_3d::rsg::Id currentPointCloudId = 0;
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","raw_point_cloud"));
//		tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
		wm->scene.addGeometricNode(sensorGroupId /*wm->getRootNodeId()*/, currentPointCloudId, tmpAttributes, newPointCloudContainer, TimeStamp(timer.getCurrentTime()));
		LOG(INFO) << "Timer: Adding to scene graph took " << timer.getElapsedTime() << "[ms]";

		/* Delete the point clouds from previous cycle (if any). Actually visualization looks nicer when deletion of old data occurs right after adding new one */
		timer.reset();
		wm->scene.deleteNode(lastPointCloudId);
		lastPointCloudId = currentPointCloudId;	// Setting hints to delet the correct data in next cycle
		LOG(INFO) << "Timer: Cleaning up scene graph took " << timer.getElapsedTime() << "[ms]";

		/* Do the hard work */
		timer.reset();
		std::vector<brics_3d::rsg::Id> inputIds;
		std::vector<brics_3d::rsg::Id> outputIds;
		inputIds.push_back(currentPointCloudId);
		perceptionBlock->setData(inputIds);
		perceptionBlock->execute();
		perceptionBlock->getData(outputIds);
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO) << "Timer: SinpleSceneAnalyzer processing time took " << timer.getElapsedTime() << "[ms]";

		timer.reset();
		if(enableDynamicROI) {
			std::vector<brics_3d::rsg::Id> roiInputIds;
			std::vector<brics_3d::rsg::Id> roiOutputIds;
			roiInputIds.resize(1);
			tmpAttributes.clear();
			tmpAttributes.push_back(Attribute("name","roi_box"));
			wm->scene.getNodes(tmpAttributes, tmpResultIds); // find node
			assert(tmpResultIds.size() == 1);
			roiInputIds[brics_3d::RoiAdapter::existingRoiBoxIdInputIndex] = tmpResultIds[0];
			roiInputIds.insert(roiInputIds.end(), outputIds.begin(), outputIds.end()); // append all result ID (however only the first one will be taken)
			LOG(INFO) << "roiInputIds.size() " << roiInputIds.size();
			roiUpdateBlock->setData(roiInputIds);
			roiUpdateBlock->execute();
			roiUpdateBlock->getData(roiOutputIds);
		}
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO) << "Timer: Dynamic ROI processing time took " << timer.getElapsedTime() << "[ms]";

		timer.reset();
		std::vector<brics_3d::rsg::Id> associationInputIds;
		associationInputIds.resize(2);
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","sensor"));
		wm->scene.getNodes(tmpAttributes, tmpResultIds); // find node
		assert(tmpResultIds.size() == 1);
		associationInputIds[brics_3d::SceneAssociation::percievedObjectsSubGraphRootIndex] = tmpResultIds[0];
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","sceneObjects"));
		wm->scene.getNodes(tmpAttributes, tmpResultIds); // find node
		assert(tmpResultIds.size() == 1);
		associationInputIds[brics_3d::SceneAssociation::sceneObjectsSubGraphRootIndex] =  tmpResultIds[0];
		sceneAssociation->setData(associationInputIds);
		sceneAssociation->execute();
		benchmark->output << timer.getElapsedTime() << " ";
		LOG(INFO) << "Timer: Assocaitiaon processing time took " << timer.getElapsedTime() << "[ms]";

		/* Some debug output */
		brics_3d::rsg::DotGraphGenerator dotGraphTraverser;
		timer.reset();
		wm->scene.executeGraphTraverser(&dotGraphTraverser, wm->scene.getRootId());
		LOG(INFO) << "Timer: Dot traveral took " << timer.getElapsedTime() << "[ms]";
		//cout << "GRAPH: "<< endl << dotGraphTraverser.getDotGraph() << endl;

		LOG(INFO) << "Total cycle time: "  << completeCycleTimer.getElapsedTime() << " [ms]" << std::endl;
		benchmark->output << completeCycleTimer.getElapsedTime() << " ";
		LOG(INFO) << "----------------------------------------" << endl;

		/* publish results as ROS message */
		publishCurrentSceneObjects();

		benchmark->output << std::endl;
		count++;
	}

	void publishCurrentSceneObjects() {

		/* setup request */
		vector<brics_3d::rsg::Attribute> queryAttributes;
		queryAttributes.push_back(sceneObjectTag);
		queryAttributes.push_back(Attribute("debugInfo","scnObj")); // some workaround for now


		/* query */
		vector<brics_3d::SceneObject> resultObjects;
		wm->getSceneObjects(queryAttributes, resultObjects);
		LOG(INFO) << "Number of found scene objects: " << resultObjects.size();

		if(resultObjects.size() <= 0) {
			return; //No objects found - so don't publish anything...
		}

		/* setup according ROS message */
		currentSceneObjects.sceneObjects.clear();
		currentSceneObjects.sceneObjects.resize(resultObjects.size());
		geometry_msgs::TransformStamped tmpTransformMsg;
		std::stringstream objectSceneFrameID;
		for (unsigned int i = 0; i < static_cast<unsigned int>(resultObjects.size()); ++i) {
			brics_3d_msgs::SceneObject tmpSceneObject;
//			tmpSceneObject.id = resultObjects[i].id;
//			tmpSceneObject.parentId = resultObjects[i].parentId;
			SceneGraphTypeCasts::convertIdToRosMsg(resultObjects[i].id, tmpSceneObject.id);
			SceneGraphTypeCasts::convertIdToRosMsg(resultObjects[i].parentId, tmpSceneObject.parentId);

			SceneGraphTypeCasts::convertTransformToRosMsg(resultObjects[i].transform, tmpTransformMsg);
			tmpTransformMsg.header.stamp = ros::Time::now();
			tmpTransformMsg.header.frame_id = rootFrameId;
			objectSceneFrameID.str("");
			objectSceneFrameID << "scene_object_" << resultObjects[i].id;
			tmpTransformMsg.child_frame_id = objectSceneFrameID.str();

			tmpSceneObject.transform = tmpTransformMsg;

			brics_3d::rsg::Box::BoxPtr tmpBox = boost::dynamic_pointer_cast<Box>(resultObjects[i].shape);

			if ( tmpBox != 0) {
				tmpSceneObject.shape.type = brics_3d_msgs::Shape::BOX;
				tmpSceneObject.shape.dimensions.resize(3);
				tmpSceneObject.shape.dimensions[0] = tmpBox->getSizeX();
				tmpSceneObject.shape.dimensions[1] = tmpBox->getSizeY();
				tmpSceneObject.shape.dimensions[2] = tmpBox->getSizeZ();
			}


			tmpSceneObject.attributes.resize(resultObjects[i].attributes.size());
			for (unsigned int j = 0; j < static_cast<unsigned int>(resultObjects[i].attributes.size()); ++j) {
				tmpSceneObject.attributes[j].key = resultObjects[i].attributes[j].key;
				tmpSceneObject.attributes[j].value = resultObjects[i].attributes[j].value;
			}

			currentSceneObjects.sceneObjects[i] = tmpSceneObject;
			if (publishDebugTF) {
				tfBroadcaster.sendTransform(tmpTransformMsg);
			}
		}

		sceneObjectPublisher.publish(currentSceneObjects);
	}


	/// Helper tool to convert point clouds
	brics_3d::PCLTypecaster converter;

	/// The world model that stores all 3D data.
	brics_3d::WorldModel* wm;

#ifdef ENABLE_OSG
	/// Visualization tool for world model
	brics_3d::rsg::OSGVisualizer* wmObserver;
#endif

	brics_3d::rsg::DotVisualizer* dbgObserver;

	/// Hint which point cloud is is from previous cycle.
	brics_3d::rsg::Id lastPointCloudId;

	/// For stats & debugging
	int count;

	/// For timings
	brics_3d::Timer timer;
	brics_3d::Timer completeCycleTimer;

	brics_3d::Benchmark* benchmark;

	/// Functional block that will do the actual work
	brics_3d::rsg::IFunctionBlock* perceptionBlock;

	/// Functional block that will set uo the ROI
	brics_3d::rsg::IFunctionBlock* roiBlock;

	/// Functional block that will update the ROI based on perception results
	brics_3d::rsg::IFunctionBlock* roiUpdateBlock;

	/// Functional block that will associate percieved and previosly stored scene objects
	brics_3d::rsg::IFunctionBlock* sceneAssociation;

	/// Function block's parameters
	brics_3d::ParameterSet algorithmParameters;

	/// The tag that will be used to tag an object as "scene object". These objects are the actually our <b>result</br>.
	brics_3d::rsg::Attribute sceneObjectTag;

	/// To this TF frame_id all objects in the world model relate. Default is the Kinect frame.
	string rootFrameId;

	/* ROS communication */
	///The result message
	brics_3d_msgs::SceneObjects currentSceneObjects;

	/// Publisher for result
	ros::Publisher sceneObjectPublisher;

	tf::TransformBroadcaster tfBroadcaster;

//	ros::ServiceServer resentSceneGraphService;

	// Program flags
	bool publishDebugTF;
	bool writeDebugGraphToFile;
	bool enableDynamicROI;
	bool enableDistributedSceneGraph;

};

int main (int argc, char** argv) {

	ros::init(argc, argv, "youbot_scene_analyzer");
	ros::NodeHandle node;

	brics_3d::WorldModel* wm = new brics_3d::WorldModel();
	SceneAnalyzer sceneAnalyzer(wm);
	brics_3d::WorldModelQueryServer wmServer(node, wm);
	wmServer.setServiceNameSpace("/worldModel/");
	wmServer.initialize();

	node.param<std::string>("worldModelRootFrameId", sceneAnalyzer.rootFrameId, "/openni_rgb_optical_frame");
	node.param<bool>("publishDebugTF", sceneAnalyzer.publishDebugTF, false);
	node.param<bool>("writeDebugGraphToFile", sceneAnalyzer.writeDebugGraphToFile, false);
	node.param<bool>("enableDynamicROI", sceneAnalyzer.enableDynamicROI, true);
	node.param<bool>("enableDistributedSceneGraph", sceneAnalyzer.enableDistributedSceneGraph, true);

	if(sceneAnalyzer.writeDebugGraphToFile) {
		sceneAnalyzer.dbgObserver = new brics_3d::rsg::DotVisualizer(&wm->scene);
		//sceneAnalyzer.dbgObserver->setKeepHistory(true); // If set to true _all_ graph snapshots will be saved as a file.
		wm->scene.attachUpdateObserver(sceneAnalyzer.dbgObserver);
	}

	if(sceneAnalyzer.enableDistributedSceneGraph) {
		brics_3d::rsg::SceneGraphROSCommunicator* updater = new brics_3d::rsg::SceneGraphROSCommunicator(node, "/brics_mm/worldModel/");
		wm->scene.attachUpdateObserver(updater);

		brics_3d::rsg::SceneGraphResentServer* resentServer = new brics_3d::rsg::SceneGraphResentServer(node, wm, updater);
		resentServer->setServiceNameSpace("/worldModel/");
		resentServer->initialize();
	}

	/*
	 * Setup the algorithm's configuration
	 */
	brics_3d::ParameterSet algorithmParameters;
	string tmpParameter;


	/* Octree Filtering */
	node.param<std::string>("/youbot_scene_analyzer/voxelSize", tmpParameter, "0.01");
	algorithmParameters << brics_3d::Parameter("voxelSize", tmpParameter);

	/* Plane segmentation */
	node.param<std::string>("/youbot_scene_analyzer/sacDistanceThreshold", tmpParameter, "0.02"); //[m]
	algorithmParameters << brics_3d::Parameter("sacDistanceThreshold", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/sacMaxIterations", tmpParameter, "1000");
	algorithmParameters << brics_3d::Parameter("sacMaxIterations", tmpParameter);

	/* Eucledion Clustering */
	node.param<std::string>("/youbot_scene_analyzer/clusterTolerance", tmpParameter, "0.02"); //[m]
	algorithmParameters << brics_3d::Parameter("clusterTolerance", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/minClusterSize", tmpParameter, "30");
	algorithmParameters << brics_3d::Parameter("minClusterSize", tmpParameter);//depends a litle bit on the octree filter size
	node.param<std::string>("/youbot_scene_analyzer/maxClusterSize", tmpParameter, "25000");
	algorithmParameters << brics_3d::Parameter("maxClusterSize", tmpParameter);//depends a litle bit on the octree filter size

	/* Boundaries for bounding box classification */
	node.param<std::string>("/youbot_scene_analyzer/boundingBoxVolumefeatureLowerLimit", tmpParameter, "0.0001");
	algorithmParameters << brics_3d::Parameter("boundingBoxVolumefeatureLowerLimit", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/boundingBoxVolumefeatureUpperLimit", tmpParameter, "0.0008");
	algorithmParameters << brics_3d::Parameter("boundingBoxVolumefeatureUpperLimit", tmpParameter);

	sceneAnalyzer.perceptionBlock->configure(algorithmParameters);


	/*
	 * Setup ROI parameters
	 */
	brics_3d::ParameterSet roiParameters;

	node.param<std::string>("/youbot_scene_analyzer/roiBoxSizeX", tmpParameter, "2.0");
	roiParameters << brics_3d::Parameter("roiBoxSizeX", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/roiBoxSizeY", tmpParameter, "2.0");
	roiParameters << brics_3d::Parameter("roiBoxSizeY", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/roiBoxSizeZ", tmpParameter, "2.0");
	roiParameters << brics_3d::Parameter("roiBoxSizeZ", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/roiCenterX", tmpParameter, "0.2");
	roiParameters << brics_3d::Parameter("roiCenterX", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/roiCenterY", tmpParameter, "0.0");
	roiParameters << brics_3d::Parameter("roiCenterY", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/roiCenterZ", tmpParameter, "1.1");
	roiParameters << brics_3d::Parameter("roiCenterZ", tmpParameter);
	node.param<std::string>("/youbot_scene_analyzer/roiPitch", tmpParameter, "0.7");
	roiParameters << brics_3d::Parameter("roiPitch", tmpParameter);

	sceneAnalyzer.roiBlock->configure(roiParameters);

	/* Add the roi to WM */
	std::vector<brics_3d::rsg::Id> roiInput;
	roiInput.push_back(sceneAnalyzer.wm->getRootNodeId());
	sceneAnalyzer.roiBlock->setData(roiInput);
	sceneAnalyzer.roiBlock->execute();

	/* Setup communication */
	ros::Subscriber inputSubscriber;
	unsigned int topicBufferSize = 1; //sould be rather small, if processing is slower than the data source to prevent congestion
//	inputSubscriber = node.subscribe<pcl::PointCloud<PointType> >("/camera/depth/points", topicBufferSize, &SceneAnalyzer::callback, &sceneAnalyzer);
	inputSubscriber = node.subscribe<pcl::PointCloud<PointType> >("/camera/rgb/points", topicBufferSize, &SceneAnalyzer::callback, &sceneAnalyzer);
	sceneAnalyzer.sceneObjectPublisher = node.advertise<brics_3d_msgs::SceneObjects>("youbot_scene_analyzer/currentSceneObjects", 1);



	ros::MultiThreadedSpinner spinner(2); // Use 2 threads
	spinner.spin();

	delete wm;

    return (0);
}

/* EOF */
