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

#include "ROSWrapper.h"
#include "graphics/collada/ColladaParser.h"
#include "algorithms/collisionChecker/pqp/CollisionCheckerPQP.h"
#include "utils/Logger.h"
#include "math/Quaternion.h"
#include "algorithms/metric/WeightedMetric.h"
#include "core/robot/PrismaticJoint.h"

using namespace std;

namespace BRICS_MM {

ROSWrapper::ROSWrapper(ros::NodeHandle& nodeHandle) : nodeHandle(nodeHandle) {
	nodeHandle.param<bool>("show_osg_visualizer", show_osg_visualizer, true/*false*/);
	nodeHandle.param<bool>("use_planning_environment", use_planning_environment, false);
	nodeHandle.param<bool>("provide_follow_trajectory_action", provide_follow_trajectory_action, true);
	this->wm = 0;
}

void ROSWrapper::init(PathPlannerComponent* pathPlanner,
	                  RobotComponent* robot,
	                  ConfigurationSpaceComponent* cspace,
	                  CartesianSpaceComponent* cartesianSpace,
					  CollisionCheckerComponent* collisionChecker) {
	this->robot = robot;
	this->cspace = cspace;
	this->pathPlanner = pathPlanner;

	this->environment = cartesianSpace;
	this->collisionChecker = collisionChecker;
	geoms = 0;


	testrobot = new RobotComponent;
	motionExecution = new ROSMotionExecution;
	motionExecution->wrapper = this;
	motionExecutionThread = 0;


	if (show_osg_visualizer) {
		visualizer = new OSGVisualizer;
		visualizer->startInteraction(); //only works with correct compile time flags
		visualizer->attachCartesianSpace(environment);

	}
	
	// get planner config
	std::string default_planner_config;
	if (nodeHandle.getParam("default_planner_config", default_planner_config)) {
		loadPlannerConfig(default_planner_config);
	}

	OmplWrapper ompl;
	ompl.init(robot, cspace, pathPlanner, environment, collisionChecker);
}

ROSWrapper::~ROSWrapper() {
	delete motionExecution;
	delete testrobot;
	delete visualizer;
	delete collisionChecker;
	delete pathPlanner;
	delete environment;
	delete robot;
	if (wm) {
		delete wm;
		wm = 0;
	}
}

void ROSWrapper::registerServices() {
	services.push_back(nodeHandle.advertiseService("GetPath", &ROSWrapper::GetPath, this));
	services.push_back(nodeHandle.advertiseService("GetMotionPlan", &ROSWrapper::GetMotionPlan, this));
	services.push_back(nodeHandle.advertiseService("Goto", &ROSWrapper::Goto, this));
	services.push_back(nodeHandle.advertiseService("LoadRobot", &ROSWrapper::LoadRobot, this));
	services.push_back(nodeHandle.advertiseService("MoveRobot", &ROSWrapper::MoveRobot, this));
	services.push_back(nodeHandle.advertiseService("SelectCSpace", &ROSWrapper::SelectCSpace, this));
	services.push_back(nodeHandle.advertiseService("SelectPlanner", &ROSWrapper::SelectPlanner, this));
	subscribers.push_back(nodeHandle.subscribe("/joint_states", 100, &ROSWrapper::callback_joint_states, this));
	subscribers.push_back(nodeHandle.subscribe("/tf", 100, &ROSWrapper::callback_tf, this));
	publisherTwist = nodeHandle.advertise<geometry_msgs::Twist>("/base_controller/command", 5);
	publisherJointTrajectory = nodeHandle.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 5);

	// subscribe to ROS planning environment, if not disabled from configuration
	// this will hold the execution until the planning environment (resp. the environment server node) is started
	collisionModelsInterface = 0;
	if (use_planning_environment) {
		collisionModelsInterface = new planning_environment::CollisionModelsInterface("robot_description");
		if (collisionModelsInterface->loadedModels())
			ROS_INFO("Loaded robot models.");
		else
			ROS_ERROR("Could not load robot models.");
	}

	// provide trajectory follower, if not disabled from configuration
	followJointTrajectoryServer = 0;
	if (provide_follow_trajectory_action) {
		followJointTrajectoryServer = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
			nodeHandle, "arm_1/arm_controller/joint_trajectory_action", boost::bind(&ROSWrapper::followJointTrajectoryCallback, this, _1), false);
		followJointTrajectoryServer->start();
	}	
}

bool ROSWrapper::GetPath(brics_mm_ros::BRICSGetPath::Request& req, brics_mm_ros::BRICSGetPath::Response& res) {
	RobotConfiguration start = req.start.position;
	LOG(WARNING) << "Got start pos: " << start;
	
	RobotConfiguration goal = req.goal.position;
	LOG(WARNING) << "Got goal pos: " << goal;

	if (wm != 0) {
		LOG(DEBUG) << "Retrieving data from scene graph.";
		BRICS_MM::GeomContainer* sceneGeoms = new BRICS_MM::GeomContainer();
		BRICS_MM::SceneGraphToGeomConverter* sceneToGeoms = new BRICS_MM::SceneGraphToGeomConverter(&wm->scene, wm->scene.getRootId(), sceneGeoms);
		wm->scene.executeGraphTraverser(sceneToGeoms, wm->scene.getRootId());
		LOG(DEBUG) << environment->getNumContainers() << " geom container(s) to be cleaned.";
		environment->clear();
		LOG(DEBUG) << environment->getNumContainers() << " geom container(s) left.";
		environment->addContainer(sceneGeoms);
		LOG(DEBUG) << environment->getNumContainers() << " geom container(s) added.";
		if(visualizer) {
			visualizer->viewAll();
		}
	}

	Path path = pathPlanner->getPath(start, goal);
	LOG(WARNING) << "Planned: " << path;

	convertPathToJointTrajectory(path, res.trajectory);
	res.errorCode = pathPlanner->getLastError();
	
	ROS_INFO("GetPath() called, returned %d points, error %d", path.getNumViaPoints(), res.errorCode);
	
	return true;
}

bool ROSWrapper::GetMotionPlan(arm_navigation_msgs::GetMotionPlan::Request& req, arm_navigation_msgs::GetMotionPlan::Response& res) {
	return false;
}

bool ROSWrapper::LoadRobot(brics_mm_ros::BRICSLoadRobot::Request& req, brics_mm_ros::BRICSLoadRobot::Response& res) {
	std::string filename = std::string(BRICS_DATA_DIR) + "/models/" + req.filename;

	ParameterSet params;
	params << Parameter("filename", filename);
	params << Parameter("addSelfCollisions", "unconnected");
	if (!robot->selectRobot(params)) {
		ROS_INFO("Could not load robot: %s", filename.c_str());
		res.errorcode = 1;
		return false;
	}

/*
	// hack: add 7th joint to Care-o-bot, which is missing in dae file
	IRobotKinematicsSetup* kin = const_cast<IRobotKinematicsSetup*>(&robot->getKinematics());
	PrismaticJoint* prismatic = new PrismaticJoint;
	prismatic->setName("arm_7_joint");
	prismatic->setAxis(Vector3(1, 0, 0));  // move along the x-axis
	prismatic->setLimits(0, 1);            // some arbitrary limits
	kin->addKinNode(prismatic);           // add it to the robot
*/
	// Select matching cspace, so clients are not required to call it manually
	brics_mm_ros::BRICSSelectCSpace::Request reqCSpace;
	brics_mm_ros::BRICSSelectCSpace::Response resCSpace;
	SelectCSpace(reqCSpace, resCSpace);

	currentConfig = robot->getConfig();
	res.errorcode = 0;

	if (visualizer) {
		visualizer->addRobot(robot, filename);
		visualizer->viewAll();
	}

	return true;
}

bool ROSWrapper::MoveRobot(brics_mm_ros::BRICSMoveRobot::Request& req, brics_mm_ros::BRICSMoveRobot::Response& res) {
	RobotConfiguration config;
	convertJointStateToConfig(req.state, config);
	robot->setConfig(config);
	return true;
}


bool ROSWrapper::Goto(brics_mm_ros::BRICSGoto::Request& req, brics_mm_ros::BRICSGoto::Response& res) {
	if (!robot)
		return false;

	RobotConfiguration start = robot->getConfig();
	LOG(WARNING) << "Start pos: " << start;
	
	RobotConfiguration goal(robot->getNumDOF());
	for (unsigned int i=0; i<req.pos.size() && i < goal.size(); i++)
		goal[i] = req.pos[i];
	LOG(WARNING) << "Goal pos: " << goal;

	Path path = pathPlanner->getPath(start, goal);
	LOG(WARNING) << "Planned: " << path;

	overSamplePath(path, *cspace->getInterpolator(), *cspace->getMetric(), 1);

/*  // example to send just one trajectory message to arm controller
	convertPathToJointPath(path, res.path);
	res.errorCode = pathPlanner->getLastError();
	
	ROS_INFO("Goto() called, returned %d points, error %d", path.getNumViaPoints(), res.errorCode);

	trajectory_msgs::JointTrajectory goalmsg;
	convertPathToJointTrajectory(path, goalmsg);
	publisherJointTrajectory.publish(goalmsg);
*/

	// example to execute motion by sending single controller messages for each point on the path
	// only start if not already one motion controller thread running
	if (!motionExecutionThread) {
		motionExecution->setRobot(robot);
		motionExecution->setPath(path);
		motionExecution->setDelay(50);
		ROS_INFO("Start to execute path with %d points.", path.getNumViaPoints());
		motionExecutionThread = new boost::thread(boost::bind(motionExecutionThreadFunction, this));			
	}

	ROS_INFO("Goto() finished.");
	return true;
}


bool ROSWrapper::SelectCSpace(brics_mm_ros::BRICSSelectCSpace::Request& req, brics_mm_ros::BRICSSelectCSpace::Response& res) {
	ParameterSet params;
	for (unsigned int i=0; i<req.paramsNames.size() && i<req.paramsValues.size(); i++)
		params[req.paramsNames[i]] = req.paramsValues[i];

	// just one example for handling parameters. Should be done in more general way
	if (nodeHandle.hasParam("metric/weights") && !params.hasParam("metric/weights")) {
		std::string s;
		nodeHandle.getParam("metric/weights", s);
		params << Parameter("metric", std::string("<weights>") + s + "</weights>");
	}

	cspace->selectCSpace(params, robot);
	return true;
}

bool ROSWrapper::SelectPlanner(brics_mm_ros::BRICSSelectPlanner::Request& req, brics_mm_ros::BRICSSelectPlanner::Response& res) {
ROS_ERROR("Called Select Planner");
	ParameterSet params;
	for (unsigned int i=0; i<req.paramsNames.size() && i<req.paramsValues.size(); i++) {
		params[req.paramsNames[i]] = req.paramsValues[i];
	LOG(WARNING) << "set param: <" << req.paramsNames[i] << ">, <" << req.paramsValues[i] << ">";
}
	if (!params.hasParam("")) {
		if (nodeHandle.hasParam("planner")) {
			std::string s;
			nodeHandle.getParam("planner", s);
			params[""] = s;
		} else {
			params[""] = "ConConRRT";
		}
	}
	params["maxConfigDistance"] = "1000";
	params["maxConnectDist"] = "1000";
	pathPlanner->setComponents(cspace, robot, collisionChecker);
	pathPlanner->selectPathPlanner(params);
	if (pathPlanner->getCurrentPathPlanner() == 0)
		ROS_INFO("No planner selected.");

	return true;
}

void ROSWrapper::callback_joint_states(const sensor_msgs::JointState::ConstPtr& js) {
	if (!robot)
		return;

	RobotConfiguration config;	
	if (!convertJointStateToConfig(*js.get(), config))
		return;

	robot->setConfig(config);
	currentConfig = config;
	if (visualizer)
		visualizer->updateRobot(robot);
	LOG(INFO) << "Received new robot config (joint_states): " << config;
}

void ROSWrapper::callback_tf(const tf::tfMessage::ConstPtr& tfmsg) {
	if (!robot)
		return;

	//find correct tf entry.
	//in this case hacked version for Care-o-bot
	for (unsigned int i=0; i<tfmsg->transforms.size(); i++) {
		geometry_msgs::TransformStamped transform = tfmsg->transforms[i];
		if (transform.child_frame_id == "/base_footprint" && transform.header.frame_id == "/odom_combined") {
			RobotConfiguration config(currentConfig);
			config[0] = transform.transform.translation.x * 1000;
			config[1] = transform.transform.translation.y * 1000;
			Quaternion quat(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
			Vector3 v(0, 0, 1);
			quat.toAxisAngle(v, config[2]);
			robot->setConfig(config);
			currentConfig = config;
			if (visualizer)
				visualizer->updateRobot(robot);
			LOG(INFO) << "Received new robot config (tf): " << config;			
		}			
	}
}

void ROSWrapper::loadEnvironment(std::string filename) {
	string pathfilename = std::string(BRICS_MODELS_DIR) + "/" + filename;
	ColladaParser collada;
	GeomContainer* newGeoms = new GeomContainer;
	if (collada.load(pathfilename, newGeoms)) {
		if (geoms)
			environment->deleteContainer(geoms);
		geoms = newGeoms;
		environment->addContainer(geoms);
		visualizer->viewAll();
	} else {
		delete newGeoms;
		ROS_INFO("Could not load environment: %s", filename.c_str());
	}
}


void ROSWrapper::fillROSHeader(std_msgs::Header& header) {
	header.seq = 0;
	header.stamp = ros::Time::now();
	header.frame_id = "0";
}

void ROSWrapper::copyJointNames(IRobotKinematicsSetup* robot, std::vector<std::string>& names) {
	names.clear();
	if (robot) {
		for (unsigned int i=0; i<robot->getNumDOF(); i++) {
			// todo add fnc to kinematicssetup			if (!robot->getKinematics().belongsToBase(getKinNode(j))
			if (i>=3) {
				const KinNode* node = robot->getKinNodeForDOF(i);
				if (node)
					names.push_back(node->getName());
				else
					names.push_back("");
			}
		}
	}
}
	
bool ROSWrapper::convertConfigToJointState(const RobotConfiguration& config, sensor_msgs::JointState& jointState) {
	fillROSHeader(jointState.header);
	copyJointNames(&robot->getKinematics(), jointState.name);
	jointState.position = config;	

	return true;
}

bool ROSWrapper::convertJointStateToConfig(const sensor_msgs::JointState& jointState, RobotConfiguration& config) {
	unsigned int dim = cspace->getDimension();
	if (jointState.position.size() != dim)
		LOG(WARNING) << "Dimension mismatch while converting JointState to config.";

	config = RobotConfiguration(dim);

	for (unsigned int i=0; i<jointState.position.size(); i++) {
		unsigned int dof = i;
		if (i < jointState.name.size()) {
			const KinNode* kinNode = robot->getKinematics().getKinNode(jointState.name[i]);
			if (kinNode && kinNode->getNumDOF() > 0)
				dof = kinNode->getDOF(0)->getIndex();
		}
		if (dof < dim)
			config[dof] = jointState.position[i];
	}

	return true;
}

bool ROSWrapper::convertPathToJointTrajectory(const Path& path, trajectory_msgs::JointTrajectory& jointTrajectory) {
	fillROSHeader(jointTrajectory.header);
	std::vector<std::string> names; 
	copyJointNames(&robot->getKinematics(), jointTrajectory.joint_names);

	if (path.getNumViaPoints() == 0)
		return true;

	unsigned int dim = path.getConfigForVia(0).size();

	for (int i=0; i<path.getNumViaPoints(); i++) {
		RobotConfiguration config = path.getConfigForVia(i);
		if (config.size() != dim) { // should not happen
			jointTrajectory.points.clear();
			return false;
		}

		trajectory_msgs::JointTrajectoryPoint jtp;
		for (unsigned int j=0; j<robot->getNumDOF(); j++)
			// todo add fnc to kinematicssetup			if (!robot->getKinematics().belongsToBase(getKinNode(j))
			if (j>=3) 
				jtp.positions.push_back(config[j]);
		// todo: set correct timing data
		jtp.time_from_start = ros::Duration(0.1) * (i+1);
		jointTrajectory.points.push_back(jtp);
	}

	return true;
} 

// Handler function for motion execution, called in a new thread.
void ROSWrapper::motionExecutionThreadFunction(ROSWrapper* obj) {
	obj->motionExecution->executeMotion();
	boost::thread* thread = obj->motionExecutionThread;
	obj->motionExecutionThread = 0;
	delete thread;
}


// Called by the motionExecution thread, requesting to update the
// robot by determining and sending the next control commands
void ROSWrapper::ROSMotionExecution::updateRobot(IRobotJointSpace* robot) {
	// control robot. Send only next config to arm controller
	RobotConfiguration config = robot->getConfig();
	Path path;
	path.addViaPoint(1.0, config);

	trajectory_msgs::JointTrajectory jointTrajectory;
	wrapper->convertPathToJointTrajectory(path, jointTrajectory);
	wrapper->publisherJointTrajectory.publish(jointTrajectory);
	if (config.size() > 5)
		ROS_INFO("updateRobot %f %f %f", config[3], config[4], config[5]);


// TODO: generate e.g. Twist message to move robot base
//	geometry_msgs::Twist twist;
//	wrapper->publisherTwist.publish(twist);
}

void ROSWrapper::followJointTrajectoryCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
//void ROSWrapper::followJointTrajectoryCallback() {
}

ParameterSet ROSWrapper::loadPlannerConfig(string configName) {
	string planner_type;
	if (!nodeHandle.getParam("planner_configs/" + configName + "/type", planner_type)) {
		ROS_ERROR("got the type: %s", planner_type.c_str());
		return ParameterSet();
	}

	ParameterSet params(planner_type);
	ParameterSet paramsPossible = pathPlanner->getPathPlannerParameters(planner_type);

	ParameterSet::iterator it;
	for (it = paramsPossible.begin(); it != paramsPossible.end(); it++) {
		string sParam;
		if (nodeHandle.getParam(it->first, sParam))
			params.add(Parameter(it->first, sParam));
	}

	return params;
}

} // namespace BRICS_MM

