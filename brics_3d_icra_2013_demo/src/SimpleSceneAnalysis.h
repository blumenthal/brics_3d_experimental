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

#ifndef SIMPLESCENEANALYSIS_H_
#define SIMPLESCENEANALYSIS_H_

#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/TriangleMeshExplicit.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/Version.h>
#include <brics_3d/algorithm/filtering/Octree.h>
#include <brics_3d/algorithm/filtering/BoxROIExtractor.h>
#include <brics_3d/algorithm/filtering/MaskROIExtractor.h>
#include <brics_3d/algorithm/featureExtraction/BoundingBox3DExtractor.h>
#include <brics_3d/algorithm/featureExtraction/PCA.h>
#include <brics_3d/algorithm/segmentation/RegionBasedSACSegmentation.h>
#include <brics_3d/algorithm/segmentation/EuclideanClustering.h>
#include <brics_3d/algorithm/segmentation/EuclideanClusteringPCL.h>
#include <brics_3d/util/PCLTypecaster.h>
#include <brics_3d/util/Timer.h>
#include <brics_3d/util/Benchmark.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/PointCloud.h>
#include <brics_3d/worldModel/sceneGraph/Mesh.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>

using brics_3d::Logger;

namespace brics_3d {

class SimpleSceneAnalysis : public rsg::IFunctionBlock {
public:
	SimpleSceneAnalysis(brics_3d::WorldModel* wmHandle);
	virtual ~SimpleSceneAnalysis();

	void configure(brics_3d::ParameterSet parameters);
	void setData(std::vector<brics_3d::rsg::Id>& inputDataIds); //NOTE: There is a contract/convention on meaning of IDs
	void execute();
	void getData(std::vector<brics_3d::rsg::Id>& newDataIds);

	void cleanUp();

private:

	/* Utils */
	brics_3d::Timer timer;
	brics_3d::Benchmark* timingBenchmark;

	/// Hint which point cloud is is from previous cycle.
	brics_3d::rsg::Id lastFilteredPointCloudId;
	std::vector<brics_3d::rsg::Id> nextCycleDeletionList;

	/// For stats & debugging
	int count;

	/* Algorithms */
	brics_3d::Octree* octreeFilter;
	brics_3d::RegionBasedSACSegmentation* sacSegmenter;
	brics_3d::EuclideanClusteringPCL* clusterSegmentator;
	brics_3d::BoundingBox3DExtractor* boundingBoxExtractor;
	brics_3d::PCA* pcaExtractor;

	/* Parameters */
	double boundingBoxVolumefeatureLowerLimit;
	double boundingBoxVolumefeatureUpperLimit;

};

}

#endif /* SIMPLESCENEANALYSIS_H_ */

/* EOF */
