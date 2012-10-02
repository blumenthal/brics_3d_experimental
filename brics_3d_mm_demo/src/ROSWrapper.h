// BRICS_MM - Mobile Manipulation Library
// Copyright (c) 2009 GPS GmbH
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

#ifndef ROSWRAPPER_H_
#define ROSWRAPPER_H_

#include "brics_mm/components/ComponentSimpleSetup.h"
#include "brics_mm/algorithms/motionExecution/SimpleMotionExecution.h"
#include "brics_mm/graphics/OSGVisualizer.h"

#include "ompl/OmplWrapper.h"

//BRICS_3D
#include <brics_3d/worldModel/WorldModel.h>
#include "SceneGraphToGeomConverter.h"

// ROS includes
#include <ros/ros.h>
#include <brics_mm_ros/BRICSGetNextConfig.h>
#include <brics_mm_ros/BRICSGetPath.h>
#include <brics_mm_ros/BRICSGoto.h>
#include <brics_mm_ros/BRICSLoadRobot.h>
#include <brics_mm_ros/BRICSMoveRobot.h>
#include <brics_mm_ros/BRICSPlanPath.h>
#include <brics_mm_ros/BRICSSelectCSpace.h>
#include <brics_mm_ros/BRICSSelectPlanner.h>
#include <brics_mm_ros/RobotConfiguration.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>

#include <sstream>

#include <geometry_msgs/Twist.h>

#include <arm_navigation_msgs/GetMotionPlan.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <planning_environment/models/collision_models_interface.h>

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

namespace brics_mm {

//! Very basic example class that shows prototypically how to integrate into ROS
//! The class keeps track of the current position of the robot by listening on
//! tf and joint_states messages.
//! A number of services is offered that trigger calls to BRICS_MM

class ROSWrapper {
public:
	//!Default constructor
	ROSWrapper(ros::NodeHandle& nodehandle);
	
	//! Destructor unregisters all services and deletes objects
	~ROSWrapper();
	
	//! Initialises this object using the given pointers to components.
	//! None of the arguments must be 0.
	void init(PathPlannerComponent* pathPlanner,
	          RobotComponent* robot,
	          ConfigurationSpaceComponent* cspace,
	          CartesianSpaceComponent* cartesianSpace,
	          CollisionCheckerComponent* collisionChecker);

	//! Register all ROS services that this class provides
	void registerServices();	

	//Service handlers
	bool GetNextConfig(brics_mm_ros::BRICSGetNextConfig::Request& req, brics_mm_ros::BRICSGetNextConfig::Response& res);
	bool GetPath(brics_mm_ros::BRICSGetPath::Request& req, brics_mm_ros::BRICSGetPath::Response& res);
	bool GetMotionPlan(arm_navigation_msgs::GetMotionPlan::Request& req, arm_navigation_msgs::GetMotionPlan::Response& res);
//	bool LoadEnvironment(brics_mm_ros::BRICSGetPath::Request& req, brics_mm_ros::BRICSGetPath::Response& res);
	bool LoadRobot(brics_mm_ros::BRICSLoadRobot::Request& req, brics_mm_ros::BRICSLoadRobot::Response& res);
	bool MoveRobot(brics_mm_ros::BRICSMoveRobot::Request& req, brics_mm_ros::BRICSMoveRobot::Response& res);
	bool SelectCSpace(brics_mm_ros::BRICSSelectCSpace::Request& req, brics_mm_ros::BRICSSelectCSpace::Response& res);
	bool SelectPlanner(brics_mm_ros::BRICSSelectPlanner::Request& req, brics_mm_ros::BRICSSelectPlanner::Response& res);
	void callback_joint_states(const sensor_msgs::JointState::ConstPtr& js);
	void callback_tf(const tf::tfMessage::ConstPtr& tfmsg);

	bool Goto(brics_mm_ros::BRICSGoto::Request& req, brics_mm_ros::BRICSGoto::Response& res);
	static void motionExecutionThreadFunction(ROSWrapper* obj);

	void loadEnvironment(std::string filename);

	ParameterSet loadPlannerConfig(std::string configName);
	
	void setWorldModel (brics_3d::WorldModel* wm) {
		this->wm = wm;
	}

private:
	//! Fill header structure
	void fillROSHeader(std_msgs::Header& header);

	//! FollowJointTrajectory action
	void followJointTrajectoryCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
//	void followJointTrajectoryCallback();

	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* followJointTrajectoryServer;	

	//! Copy names of joints of a robot to string vector, such as used often in ROS messages
	void copyJointNames(IRobotKinematicsSetup* robot, std::vector<std::string>& names);

	//Conversion functions
	bool convertConfigToJointState(const RobotConfiguration& config, sensor_msgs::JointState& jointState);
	bool convertJointStateToConfig(const sensor_msgs::JointState& jointState, RobotConfiguration& config);
	bool convertPathToJointTrajectory(const Path& path, trajectory_msgs::JointTrajectory& jointTrajectory);

	//ROS service objects
	ros::NodeHandle& nodeHandle;
	std::vector<ros::ServiceServer> services;
	std::vector<ros::Subscriber> subscribers;
	ros::Publisher publisherTwist;
	ros::Publisher publisherJointTrajectory;

	bool show_osg_visualizer;
	bool use_planning_environment;
	bool provide_follow_trajectory_action;

	planning_environment::CollisionModelsInterface* collisionModelsInterface;

	//internal objects
	ComponentSimpleSetup components;
	GeomContainer* geoms;
	CartesianSpaceComponent* environment;
	CollisionCheckerComponent* collisionChecker;
	RobotComponent* robot;
	ConfigurationSpaceComponent* cspace;
	PathPlannerComponent* pathPlanner;
	OSGVisualizer* visualizer;

	class ROSMotionExecution : public SimpleMotionExecution {
	public:
		void updateRobot(IRobotJointSpace* robot);
		ROSWrapper* wrapper;
	};
	RobotComponent* testrobot;

	RobotConfiguration startConfig;
	RobotConfiguration goalConfig;
	RobotConfiguration currentConfig;

	ROSMotionExecution* motionExecution;
	boost::thread* motionExecutionThread;	

	brics_3d::WorldModel* wm;
};

} // namespace BRICS_MM

#endif // ROSWRAPPER_H_

