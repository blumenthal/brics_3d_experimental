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

#include "SimpleSceneAnalysis.h"

using namespace brics_3d::rsg;

namespace brics_3d {

SimpleSceneAnalysis::SimpleSceneAnalysis(brics_3d::WorldModel* wmHandle) :
	IFunctionBlock(wmHandle) {

	/* create all necessary objects */
	octreeFilter = new brics_3d::Octree();
	clusterSegmentator = new brics_3d::EuclideanClusteringPCL();
	sacSegmenter = new brics_3d::RegionBasedSACSegmentation();
	boundingBoxExtractor = new brics_3d::BoundingBox3DExtractor();
	pcaExtractor = new brics_3d::PCA();
	timingBenchmark = new brics_3d::Benchmark("simple_scene_analysis_timing");

	timingBenchmark->output << "# Subsampling, Box ROI Extraction, Dominant plane segmentation, Clustering, Cluster evaluation" << endl;

	/* defaults */
	octreeFilter->setVoxelSize(0.01); //[m] 0.005

	sacSegmenter->setDistanceThreshold(0.02);
	sacSegmenter->setMaxIterations(1000);
	sacSegmenter->setMethodType(sacSegmenter->SAC_RANSAC);
	sacSegmenter->setModelType(sacSegmenter->OBJMODEL_PLANE);
	sacSegmenter->setProbability(0.99);

	clusterSegmentator->setClusterTolerance(0.02); // [m]
	clusterSegmentator->setMinClusterSize(30/*100*/); //depens a litle bit on the octree filter
	clusterSegmentator->setMaxClusterSize(25000);

	boundingBoxVolumefeatureLowerLimit = 0.0001;
	boundingBoxVolumefeatureUpperLimit = 0.0008;

}

SimpleSceneAnalysis::~SimpleSceneAnalysis() {

	/* clean up all objects */
	delete octreeFilter;
	delete sacSegmenter;
	delete clusterSegmentator;
	delete boundingBoxExtractor;
	delete pcaExtractor;
	delete timingBenchmark;
}

void SimpleSceneAnalysis::configure(brics_3d::ParameterSet parameters) {
	double parameterValued = 0.0;
	int parameterValuei = 0;

	/* Octree */
	if (parameters.hasDouble("voxelSize", parameterValued)) {
		octreeFilter->setVoxelSize(parameterValued);
		LOG(INFO) << "Setting parameter voxelSize to " << parameterValued;
	}

	/* Plane segmantation */
	if (parameters.hasDouble("sacDistanceThreshold", parameterValued)) {
		sacSegmenter->setDistanceThreshold(parameterValued);
		LOG(INFO) << "Setting parameter sacDistanceThreshold to " << parameterValued;
	}
	if (parameters.hasInt("sacMaxIterations", parameterValuei)) {
		sacSegmenter->setMaxIterations(parameterValuei);
		LOG(INFO) << "Setting parameter sacMaxIterations to " << parameterValuei;
	}

	/* Eucledion Clustering */
	if (parameters.hasDouble("clusterTolerance", parameterValued)) {
		clusterSegmentator->setClusterTolerance(parameterValued); // [m]
		LOG(INFO) << "Setting parameter clusterTolerance to " << parameterValued;
	}
	if (parameters.hasInt("minClusterSize", parameterValuei)) {
		clusterSegmentator->setMinClusterSize(parameterValuei); //depends a litle bit on the octree filter
		LOG(INFO) << "Setting parameter minClusterSize to " << parameterValuei;
	}
	if (parameters.hasInt("maxClusterSize", parameterValuei)) {
		clusterSegmentator->setMaxClusterSize(parameterValuei);
		LOG(INFO) << "Setting parameter maxClusterSize to " << parameterValuei;
	}

	/* Boundaries for bounding box classification */
	if (parameters.hasDouble("boundingBoxVolumefeatureLowerLimit", parameterValued)) {
		boundingBoxVolumefeatureLowerLimit = parameterValued;
		LOG(INFO) << "Setting parameter boundingBoxVolumefeatureLowerLimit to " << parameterValued;
	}
	if (parameters.hasDouble("boundingBoxVolumefeatureUpperLimit", parameterValued)) {
		boundingBoxVolumefeatureUpperLimit = parameterValued;
		LOG(INFO) << "Setting parameter boundingBoxVolumefeatureUpperLimit to " << parameterValued;
	}
}

void SimpleSceneAnalysis::setData(std::vector<unsigned int>& inputDataIds){
	this->inputDataIds = inputDataIds; //make a copy
}

void SimpleSceneAnalysis::execute(){
	LOG(INFO) << "SimpleSceneAnalysis: Executing a new cycle.";
	if (inputDataIds.size() < 1) {
		LOG(WARNING) << "SimpleSceneAnalysis: Not enough input IDs.";
		return;
	}

	outputDataIds.clear();
	vector<brics_3d::rsg::Attribute> tmpAttributes;

	/* retrive a proper point cloud */
	Shape::ShapePtr inputShape;
	TimeStamp inputTime;
	unsigned int pointCloudId = inputDataIds[0];
	wm->scene.getGeometry(pointCloudId, inputShape, inputTime);// retrieve a point cloud
	rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr inputPointCloudContainer(new rsg::PointCloud<brics_3d::PointCloud3D>());
	inputPointCloudContainer = boost::dynamic_pointer_cast<PointCloud<brics_3d::PointCloud3D> >(inputShape);

	if(inputPointCloudContainer == 0) {
		LOG(ERROR) << "SimpleSceneAnalysis: Input data at inputDataIds[0] does not contain a point cloud.";
		return;
	}

	vector<unsigned int> parentIds;
	wm->scene.getNodeParents(pointCloudId, parentIds);
	assert(parentIds.size() >= 1);
//	unsigned int dataParentId = wm->getRootNodeId();
	unsigned int dataParentId = parentIds[0]; // here we take the first, however this ID might be better an input parameter to dissolve disambiguities




#if 0
	/*** SUBSAMPLING ***/


	/* subsample the point cloud */
	brics_3d::PointCloud3D::PointCloud3DPtr subsampledPointCloud(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr subsampledPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	subsampledPointCloudContainer->data=subsampledPointCloud;
	timer.reset();
	octreeFilter->filter(inputPointCloudContainer->data.get(), subsampledPointCloudContainer->data.get());
	timingBenchmark->output << timer.getElapsedTime() << "\t";
	LOG(INFO) << "Timer: Subsampling took " << timer.getElapsedTime() << "[ms]";

	unsigned int subsampledPointCloudId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","subsampled_point_cloud"));
//		tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	wm->scene.addGeometricNode(dataParentId, subsampledPointCloudId, tmpAttributes, subsampledPointCloudContainer, TimeStamp(timer.getCurrentTime()));
//		nextCycleDeletionList.push_back(subsampledPointCloudId);
#endif

	/*** ROI EXTRACTION ***/

	/* query world model for relevant box ROI data */
	vector<unsigned int> resultIds;
	Shape::ShapePtr resultShape;
	TimeStamp resultTime;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","roi_box"));
	wm->scene.getNodes(tmpAttributes, resultIds); // find node
	assert(resultIds.size() == 1);
	unsigned int boxResultId = resultIds[0];
	LOG(DEBUG) << "Found ID for label roi_box " << boxResultId;

	wm->scene.getGeometry(boxResultId, resultShape, resultTime); // retrieve geometric data
	brics_3d::rsg::Box::BoxPtr resultBox;
	resultBox = boost::dynamic_pointer_cast<brics_3d::rsg::Box>(resultShape);
	assert(resultBox != 0);
	brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new brics_3d::HomogeneousMatrix44());
//	wm->scene.getTransformForNode(boxResultId, wm->scene.getRootId(), brics_3d::rsg::TimeStamp(timer.getCurrentTime()), transform); // get transform data to that node
	wm->scene.getTransformForNode(boxResultId, pointCloudId, brics_3d::rsg::TimeStamp(timer.getCurrentTime()), transform); // needs to be raleative to the sensor/point cloud frame...


	/* Create a point cloud based on a previously stored box ROI */
	brics_3d::BoxROIExtractor boxFilter(resultBox->getSizeX(),resultBox->getSizeY(),resultBox->getSizeZ()); // NOTE: each value describes range [origin-value/2, origin+value/2]
	brics_3d::PointCloud3D::PointCloud3DPtr boxROIPointCloud(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr boxROIPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	boxROIPointCloudContainer->data = boxROIPointCloud;
	timer.reset();
	boxFilter.setBoxOrigin(transform);
		boxFilter.filter(inputPointCloudContainer->data.get(), boxROIPointCloudContainer->data.get()); //work on raw data
//	boxFilter.filter(subsampledPointCloudContainer->data.get(), boxROIPointCloudContainer->data.get()); //work on subsampled data
	LOG(INFO) << "ROI has " << boxROIPointCloudContainer->data->getSize() << " points left from " << inputPointCloudContainer->data->getSize() << " points.";
	timingBenchmark->output << timer.getElapsedTime() << "\t";
	LOG(INFO) << "Timer: ROI filtering took " << timer.getElapsedTime() << "[ms]";
	//		boxROIPointCloudContainer->data->storeToTxtFile("roi_box_filtered_point_cloud.txt");

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","roi_box_filtered_point_cloud"));
//		tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	unsigned int currentFilteredPointCloudId = 0;
	wm->scene.addGeometricNode(dataParentId, currentFilteredPointCloudId, tmpAttributes, boxROIPointCloudContainer, TimeStamp(timer.getCurrentTime()));
	wm->scene.deleteNode(lastFilteredPointCloudId);

	/*** SUBSAMPLING ***/


	/* subsample the point cloud */
	brics_3d::PointCloud3D::PointCloud3DPtr subsampledPointCloud(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr subsampledPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	subsampledPointCloudContainer->data=subsampledPointCloud;
	timer.reset();
//	octreeFilter->filter(inputPointCloudContainer->data.get(), subsampledPointCloudContainer->data.get()); //raw input
	octreeFilter->filter(boxROIPointCloudContainer->data.get(), subsampledPointCloudContainer->data.get()); //input from ROI
	LOG(INFO) << "Subsampled cloud has " << subsampledPointCloudContainer->data->getSize() << " points";
	timingBenchmark->output << timer.getElapsedTime() << "\t";
	LOG(INFO) << "Timer: Subsampling took " << timer.getElapsedTime() << "[ms]";

	unsigned int subsampledPointCloudId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","subsampled_point_cloud"));
//		tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	wm->scene.addGeometricNode(dataParentId, subsampledPointCloudId, tmpAttributes, subsampledPointCloudContainer, TimeStamp(timer.getCurrentTime()));
//		nextCycleDeletionList.push_back(subsampledPointCloudId);

	if(subsampledPointCloudContainer->data->getSize() <= 10) {
		LOG(WARNING) << "SimpleSceneAnalysis: Not enough points left after filtering. Skipping here.";
		cleanUp();
		nextCycleDeletionList.push_back(subsampledPointCloudId); //some hack so make it look nicer...
		lastFilteredPointCloudId = currentFilteredPointCloudId; // we should not for get to handle this even in the error case
		return;
	}

	/*** PLANE SEGMENTATION ***/

	/* Get the dominant plane */
	timer.reset();
	Eigen::VectorXd modelCoefficients;
	std::vector<int> inliers;

	sacSegmenter->setPointCloud(subsampledPointCloudContainer->data.get());
	sacSegmenter->segment();
	sacSegmenter->getInliers(inliers);
	sacSegmenter->getModelCoefficients(modelCoefficients);

	timingBenchmark->output << timer.getElapsedTime() << "\t";
	LOG(INFO) << "Timer: Plane segmantation took " << timer.getElapsedTime() << "[ms]";
	LOG(INFO) <<"Found Inliers: " << inliers.size();
	LOG(INFO) << "The model-coefficients are: " << endl << modelCoefficients;

	brics_3d::MaskROIExtractor indicesExtractor;
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloudWithoutDominantPlane(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pointCloudWithoutDominantPlaneContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	pointCloudWithoutDominantPlaneContainer->data=pointCloudWithoutDominantPlane;

	timer.reset();
	indicesExtractor.setMask(&inliers);
	indicesExtractor.setUseInvertedMask(true); //we don't want the plane, we want the rest
//	indicesExtractor.filter(boxROIPointCloudContainer->data.get(), pointCloudWithoutDominantPlaneContainer->data.get()); //version: subsampling first
	indicesExtractor.filter(subsampledPointCloudContainer->data.get(), pointCloudWithoutDominantPlaneContainer->data.get()); //version: ROI extraction first
	LOG(INFO) << "Timer: Mask filtering took " << timer.getElapsedTime() << "[ms]";

	cleanUp();
	nextCycleDeletionList.push_back(subsampledPointCloudId); //some hack so make it look nicer...

	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_without_dominant_plane"));
	tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	unsigned int currentSegmentedPointCloudId = 0;
	wm->scene.addGeometricNode(dataParentId, currentSegmentedPointCloudId, tmpAttributes, pointCloudWithoutDominantPlaneContainer, TimeStamp(timer.getCurrentTime()));
	nextCycleDeletionList.push_back(currentSegmentedPointCloudId);

//#if 0
	/*** Optional: BBX + TF for dominant plane ***/
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloudDominantPlane(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pointCloudDominantPlaneContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	pointCloudDominantPlaneContainer->data=pointCloudDominantPlane;
	indicesExtractor.setUseInvertedMask(false);
	indicesExtractor.filter(subsampledPointCloudContainer->data.get(), pointCloudDominantPlaneContainer->data.get());
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_dominant_plane"));
	tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	unsigned int planePointCloudId = 0;
	wm->scene.addGeometricNode(dataParentId, planePointCloudId, tmpAttributes, pointCloudDominantPlaneContainer, TimeStamp(timer.getCurrentTime()));
	nextCycleDeletionList.push_back(planePointCloudId);

	/* get bounding box */
	brics_3d::Point3D resultPlaneCenter;
	brics_3d::Vector3D resultPlaneDimensions;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr planeTransform(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 0,0,0));
	boundingBoxExtractor->computeOrientedBoundingBox(pointCloudDominantPlaneContainer->data.get(), planeTransform.get(), resultPlaneDimensions); //overrides old transform obove
	brics_3d::rsg::Box::BoxPtr planeBoundingBox(new brics_3d::rsg::Box(resultPlaneDimensions.getX(), resultPlaneDimensions.getY(), resultPlaneDimensions.getZ()/2.0));
	unsigned int tfPlaneBBoxId = 0;
	unsigned int planeBBoxId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","plane_tf"));
//	tmpAttributes.push_back(Attribute("transformType","static"));
	tmpAttributes.push_back(Attribute("taskType","sceneObject"));
//	tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	wm->scene.addTransformNode(dataParentId, tfPlaneBBoxId, tmpAttributes, planeTransform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","plane_box"));
	tmpAttributes.push_back(Attribute("shapeType","Box"));
	wm->scene.addGeometricNode(tfPlaneBBoxId, planeBBoxId, tmpAttributes, planeBoundingBox, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
	nextCycleDeletionList.push_back(tfPlaneBBoxId);

	brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
	brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
	newMeshContainer->data = newMesh;
	newMeshContainer->data->addTriangle(brics_3d::Point3D(resultPlaneDimensions.getX()/2.0,resultPlaneDimensions.getY()/2.0,0),
			brics_3d::Point3D(resultPlaneDimensions.getX()/2.0,-resultPlaneDimensions.getY()/2.0,0),
			brics_3d::Point3D(-resultPlaneDimensions.getX()/2.0,-resultPlaneDimensions.getY()/2.0,0));
	newMeshContainer->data->addTriangle(brics_3d::Point3D(-resultPlaneDimensions.getX()/2.0,-resultPlaneDimensions.getY()/2.0,0),
			brics_3d::Point3D(-resultPlaneDimensions.getX()/2.0,resultPlaneDimensions.getY()/2.0,0),
			brics_3d::Point3D(resultPlaneDimensions.getX()/2.0,resultPlaneDimensions.getY()/2.0,0));
	unsigned int planeMeshId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","plane_mesh"));
	tmpAttributes.push_back(Attribute("shapeType","Mesh"));
	wm->scene.addGeometricNode(tfPlaneBBoxId, planeMeshId, tmpAttributes, newMeshContainer, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
//#endif

	/*** CLUSTERING ***/

	if(pointCloudWithoutDominantPlaneContainer->data->getSize() <= 1) {
		LOG(WARNING) << "SimpleSceneAnalysis: Not enough points ("<< pointCloudWithoutDominantPlaneContainer->data->getSize() << ") left for clustring. Skipping here.";
		cleanUp();
		lastFilteredPointCloudId = currentFilteredPointCloudId; // we should not for get to handle this even in the error case
		return;
	}

	/* Eucledian CLustering */
	clusterSegmentator->setPointCloud(pointCloudWithoutDominantPlaneContainer->data.get());
	timer.reset();
	clusterSegmentator->segment();
	vector<brics_3d::PointCloud3D*> extractedClusters;
	clusterSegmentator->getExtractedClusters(extractedClusters);

	timingBenchmark->output << timer.getElapsedTime() << "\t";
	LOG(INFO) <<"Timer: Cluster extraction took " << timer.getElapsedTime() << "[ms]" ;
	LOG(INFO) << extractedClusters.size() << " clusters found.";

	/* add all clusters to wm */
	unsigned int clusterGroupId = 0;
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("name","point_cloud_clusters"));
//		tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
	wm->scene.addGroup(dataParentId, clusterGroupId, tmpAttributes);
	nextCycleDeletionList.push_back(clusterGroupId); // we will delete all the clusters within next cycle

	stringstream name;
	unsigned int currentClusterId = 0;
	timer.reset();
	for (unsigned int i = 0; i < extractedClusters.size(); ++i) {
		brics_3d::PointCloud3D::PointCloud3DPtr clusterCloud(extractedClusters[i]);
		brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr clusterCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
		clusterCloudContainer->data=clusterCloud;
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","cluster"));
		wm->scene.addGeometricNode(clusterGroupId, currentClusterId, tmpAttributes, clusterCloudContainer, TimeStamp(timer.getCurrentTime())); // TODO: should they have the same timestamp?

//			name.str("");
//			name << "robocup_test_cluster_" << currentClusterId << ".ply";
//			clusterCloudContainer->data->storeToPlyFile(name.str());

		/* get bounding box */
		brics_3d::Point3D resultBoxCenter;
		brics_3d::Vector3D resultBoxDimensions;
//			boundingBoxExtractor->computeBoundingBox(clusterCloudContainer->data.get(), resultBoxCenter, resultBoxDimensions);
		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr clusterTransform(new brics_3d::HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, resultBoxCenter.getX(),resultBoxCenter.getY(),resultBoxCenter.getZ()));
		boundingBoxExtractor->computeOrientedBoundingBox(clusterCloudContainer->data.get(), clusterTransform.get(), resultBoxDimensions); //overrides old transform obove
		double boundingBoxVolume = resultBoxDimensions.getX() * resultBoxDimensions.getY() * resultBoxDimensions.getZ();
		brics_3d::rsg::Box::BoxPtr clusterBoundingBox(new brics_3d::rsg::Box(resultBoxDimensions.getX(), resultBoxDimensions.getY(), resultBoxDimensions.getZ()));
		LOG(INFO) << "BoundingBox: center = " << resultBoxCenter << " dimensions = " << resultBoxDimensions;
		LOG(INFO) << "BoundingBoxVolume = " << boundingBoxVolume << "m³";

		/* add TF + Box */
		unsigned int tfBBoxId = 0;
		tmpAttributes.clear();
		tmpAttributes.push_back(Attribute("name","cluster_tf"));
//			tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
		wm->scene.addTransformNode(clusterGroupId, tfBBoxId, tmpAttributes, clusterTransform, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

		if ( (boundingBoxVolume > boundingBoxVolumefeatureLowerLimit) && (
				boundingBoxVolume < boundingBoxVolumefeatureUpperLimit)) {

			unsigned int bBoxId = 0;
			tmpAttributes.clear();
			tmpAttributes.push_back(Attribute("name","cluster_bbox"));
			tmpAttributes.push_back(Attribute("shapeType","Box"));
			tmpAttributes.push_back(Attribute("taskType","sceneObject"));
			if(count%2 == 0) { //togle bbox visualisation
//				tmpAttributes.push_back(Attribute("debugInfo","no_visualization"));
			}
			wm->scene.addGeometricNode(tfBBoxId, bBoxId, tmpAttributes, clusterBoundingBox, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
			outputDataIds.push_back(bBoxId);

			/* we also want to attach the sceneObject attribute to the parent transform */
			vector<brics_3d::rsg::Attribute> tmpTranformAttributes;
			wm->scene.getNodeAttributes(tfBBoxId, tmpTranformAttributes);
			tmpTranformAttributes.push_back(Attribute("taskType","sceneObject"));
//			tmpTranformAttributes.push_back(Attribute("taskType","percievedSceneObject"));
			wm->scene.setNodeAttributes(tfBBoxId, tmpTranformAttributes);

		}
		/* try to get rotation of cluster */
//			Eigen::MatrixXd eigenvectors;
//			Eigen::VectorXd eigenvalues;
//			pcaExtractor->computePrincipleComponents(clusterCloudContainer->data.get(), eigenvectors, eigenvalues);
//			brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr clusterRotation(new brics_3d::HomogeneousMatrix44());
//			pcaExtractor->computeRotationMatrix(eigenvectors, eigenvalues, clusterRotation.get());
//
//			unsigned int rotBBoxId = 0;
//			tmpAttributes.clear();
//			tmpAttributes.push_back(Attribute("name","cluster_pca_tf"));
//			wm->scene.addTransformNode(tfBBoxId, rotBBoxId, tmpAttributes, clusterRotation, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));
//			//wm->scene.addGeometricNode(rotBBoxId, bBoxId, tmpAttributes, clusterBoundingBox, brics_3d::rsg::TimeStamp(timer.getCurrentTime()));

	}
	timingBenchmark->output << timer.getElapsedTime() << "\t" <<endl;
	LOG(INFO) <<"Timer: Cluster evaluation took " << timer.getElapsedTime() << "[ms]" ;

	/* Setting hints to delet the correct data in next cycle */
	lastFilteredPointCloudId = currentFilteredPointCloudId;

	count++;
}

void SimpleSceneAnalysis::getData(std::vector<unsigned int>& newDataIds){
	newDataIds = this->outputDataIds;
}

void SimpleSceneAnalysis::cleanUp() {
	for (unsigned int i = 0; i < nextCycleDeletionList.size(); ++i) {
		if (wm != 0) {
			wm->scene.deleteNode(nextCycleDeletionList[i]);
		}
	}
	nextCycleDeletionList.clear();
}

}

/* EOF */
