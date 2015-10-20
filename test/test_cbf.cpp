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

namespace po = boost::program_options;

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

double sgn(const double x) {
	if (x < 0) return -1;
	else return 1;
}

int main(int argc, char *argv[]) {
	std::string tip_frame;
	ros::init (argc, argv, "test_cbf");
	parse_arguments(argc, argv, tip_frame);

	ros::NodeHandle nh;

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
	auto jsp = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	// init joint_state message
	auto js_msg = init_message(kdl_chain);

	CBF::FloatVector initial(7);
	initial << 0.51901399, -0.76299778, -0.03791721, 0.37316020, 0.02042723, 1.84549417, 0.21051285;
	joints->set(initial);

	// run controller
	ros::Rate rate(50); // 50 hz update rate
	while (ros::ok()) {
		// do some nice target motion
		double t = ros::Time::now().toSec();
		const double w = 2*M_PI / 10; // motion frequency for period 10s
		target_vector << 0.4, 0.8, 0.1,  0, 0, angles::from_degrees(30)*sgn(sin(w*t));
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

	return EXIT_SUCCESS;
}
