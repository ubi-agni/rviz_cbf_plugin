/* ============================================================
 *
 * Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the BSD license.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *   Bielefeld University, Germany
 *
 * ============================================================ */

#define CBF_NDEBUG 1

#include <cbf/primitive_controller.h>
#include <cbf/dummy_reference.h>
#include <cbf/dummy_resource.h>
#include <cbf/kdl_transforms.h>
#include <cbf/composite_transform.h>
#include <cbf/composite_potential.h>
#include <cbf/square_potential.h>
#include <cbf/axis_angle_potential.h>
#include <cbf/generic_transform.h>

#include <string>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <boost/program_options.hpp>
#include <boost/make_shared.hpp>
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/rdf_loader/rdf_loader.h>
#include <angles/angles.h>

#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

namespace po = boost::program_options;
namespace vm = visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
vm::InteractiveMarkerFeedback marker_feedback;

static void usage (const char* prog_name, const po::options_description &opts) {
	std::cout << "usage: Server [options] FILES" << std::endl;
	std::cout << opts << std::endl;
}

static void parse_arguments(int argc, char **argv,
                            std::string &tip_frame) {
	po::options_description options_description("Allowed options");
	options_description.add_options()
		("help", "show this help message")
		("tip", po::value<std::string>(&tip_frame)->required(),
		 "tip frame")
		;

	po::variables_map variables_map;
	try {
		po::store(po::parse_command_line(argc, argv, options_description),
		          variables_map);
		po::notify(variables_map);
	} catch (const po::error  &e) {
		std::cerr << e.what() << std::endl;
		usage(argv[0], options_description);
		exit (EXIT_FAILURE);
	}

	if (variables_map.count("help")) {
		usage(argv[0], options_description);
		exit (EXIT_SUCCESS);
	}
}

CBF::PrimitiveControllerPtr
createController (boost::shared_ptr<KDL::Chain> chain)
{
	unsigned int nJoints = chain->getNrOfJoints();

	// reference
	auto mReference = boost::make_shared<CBF::DummyReference>(1,6);
	// joint angle resource
	auto mResource = boost::make_shared<CBF::DummyResource>(nJoints);

	// sensor transform for position + axis angle control
	auto sensorTrafo = boost::make_shared<CBF::KDLChainPoseSensorTransform>(chain);

	// potential
	CBF::PotentialPtr pos_potential(new CBF::SquarePotential(3, 1.)); pos_potential->set_max_gradient_step_norm(0.01);
	CBF::PotentialPtr rot_potential(new CBF::AxisAnglePotential(1.)); rot_potential->set_max_gradient_step_norm(angles::from_degrees(1));
	std::vector<CBF::PotentialPtr> potentials
	      = boost::assign::list_of(pos_potential)(rot_potential);

	auto potential = boost::make_shared<CBF::CompositePotential>(potentials);
	auto solver = boost::make_shared<CBF::ThresholdGenericEffectorTransform>(6, nJoints);
	solver->setThreshold(0.1);

	// controller
	return boost::make_shared<CBF::PrimitiveController>(
	         1.0,
	         std::vector<CBF::ConvergenceCriterionPtr>(),
	         mReference,
	         potential,
	         sensorTrafo,
	         solver,
	         std::vector<CBF::SubordinateControllerPtr>(),
	         CBF::CombinationStrategyPtr(new CBF::AddingStrategy),
	         mResource
	         );
}

sensor_msgs::JointState init_message(const KDL::Chain &chain) {
	sensor_msgs::JointState msg;
	for (unsigned int i=0; i < chain.getNrOfSegments(); ++i) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		if (joint.getType() == KDL::Joint::JointType::None) continue;
		msg.name.push_back(joint.getName());
		msg.position.push_back(0);
	}
	return msg;
}

void update_message(sensor_msgs::JointState &msg,
                    const boost::shared_ptr<CBF::DummyResource> &resource) {
	msg.header.stamp = ros::Time::now();
	Eigen::Map<Eigen::VectorXd> wrapper(msg.position.data(), msg.position.size());
	wrapper = resource->get();
}

void processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
	marker_feedback = *feedback;
//	std::cout << marker_feedback << std::endl;
}

void make6DofMarker(const geometry_msgs::PoseStamped &stamped)
{
  vm::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.pose = stamped.pose;
  int_marker.name = "6dof marker";
  double scale = int_marker.scale = 0.2;

  std_msgs::ColorRGBA color;
  color.r = 0; color.g = 1; color.b = 1; color.a = 0.5;
  robot_interaction::addViewPlaneControl(int_marker, scale * 0.25, color,
                                         true, true);
  robot_interaction::addPositionControl(int_marker, false);
  robot_interaction::addOrientationControl(int_marker, false);


  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  server->applyChanges();
}

void assign (Eigen::Ref<Eigen::Vector3d> result, const geometry_msgs::Point &p) {
	result << p.x, p.y, p.z;
}
void assign (Eigen::Ref<Eigen::Vector3d> result, const geometry_msgs::Quaternion &q) {
	result << q.x, q.y, q.z;
	if (result.isMuchSmallerThan(1)) {
		result = Eigen::Vector3d::Zero();
	} else {
		double angle = 2. * acos(q.w);
		result *= angle / sin(0.5 * angle);
	}
}

int main(int argc, char *argv[]) {
	std::string tip_frame;
	ros::init (argc, argv, "run_cbf_test");
	parse_arguments(argc, argv, tip_frame);

	rdf_loader::RDFLoader rdf("robot_description");
	boost::shared_ptr<srdf::Model> srdf = rdf.getSRDF();
	if (!srdf) srdf.reset(new srdf::Model());
	const boost::shared_ptr<urdf::ModelInterface>& urdf = rdf.getURDF();
	if (!urdf) {
		ROS_ERROR("couldn't load URDF model");
		return EXIT_FAILURE;
	}
	robot_model::RobotModelPtr robot_model(new robot_model::RobotModel(urdf, srdf));

	// fetch KDL tree
	KDL::Tree kdl_tree;
	if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree)) {
		ROS_ERROR("Could not initialize KDL tree");
		return EXIT_FAILURE;
	}

	KDL::Chain kdl_chain;
	if (!kdl_tree.getChain(kdl_tree.getRootSegment()->first, tip_frame, kdl_chain)) {
		ROS_ERROR_STREAM("Could not find chain to " << tip_frame);
		return EXIT_FAILURE;
	}

	// create controller
	auto controller = createController(boost::make_shared<KDL::Chain>(kdl_chain));
	CBF::DummyResourcePtr  joints = boost::dynamic_pointer_cast<CBF::DummyResource>(controller->resource());
	CBF::DummyReferencePtr target = boost::dynamic_pointer_cast<CBF::DummyReference>(controller->reference());
	CBF::FloatVector target_vector(target->dim());

	// joint state publisher
	ros::NodeHandle nh;
	auto jsp = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	// init joint_state message
	auto js_msg = init_message(kdl_chain);


	server.reset( new interactive_markers::InteractiveMarkerServer("cbf_control","",false) );
	// initialize marker with end-effector pose from forward kinematics
	KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(kdl_chain);
	KDL::JntArray kdl_joints = KDL::JntArray(kdl_chain.getNrOfJoints());
	KDL::Frame kdl_pose;
	fk.JntToCart(kdl_joints, kdl_pose);

	tf::Pose tf_pose;
	tf::poseKDLToTF(kdl_pose, tf_pose);

	geometry_msgs::PoseStamped stamped;
	stamped.header.frame_id = kdl_tree.getRootSegment()->first;
	tf::poseTFToMsg(tf_pose, stamped.pose);

	make6DofMarker(stamped);

	// set initial pose
	marker_feedback.pose = stamped.pose;

	// run controller
	ros::Rate rate(50); // 50 hz update rate
	while (ros::ok()) {
		// set target from marker feedback
		assign(target_vector.head(3), marker_feedback.pose.position);
		assign(target_vector.tail(3), marker_feedback.pose.orientation);
		target->set_reference(target_vector);

		// perform controller step
		controller->step();

		// fill + publish ros joint_state message
		update_message(js_msg, joints);
		jsp.publish(js_msg);

		// process ros messages
		ros::spinOnce();
		rate.sleep();
	}

	server.reset();

	return EXIT_SUCCESS;
}
