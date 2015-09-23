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
	po::store(po::parse_command_line(argc, argv, options_description),
	          variables_map);
	po::notify(variables_map);

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
	std::vector<CBF::SensorTransformPtr> sensorTrafos = boost::assign::list_of
	                                                    (CBF::SensorTransformPtr(new CBF::KDLChainPositionSensorTransform(chain)))
	                                                    (CBF::SensorTransformPtr(new CBF::KDLChainAxisAngleSensorTransform(chain)));
	auto sensorTrafo = boost::make_shared<CBF::CompositeSensorTransform>(sensorTrafos);

	// potential
	std::vector<CBF::PotentialPtr> potentials = boost::assign::list_of
	                                            (CBF::PotentialPtr(new CBF::SquarePotential(3,0.1)))
	                                            (CBF::PotentialPtr(new CBF::AxisAnglePotential(0.1)));

	auto potential = boost::make_shared<CBF::CompositePotential>(potentials);

	// controller
	return boost::make_shared<CBF::PrimitiveController>(
	         1.0,
	         std::vector<CBF::ConvergenceCriterionPtr>(),
	         mReference,
	         potential,
	         sensorTrafo,
	         boost::make_shared<CBF::DampedGenericEffectorTransform>(6, nJoints),
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

int main(int argc, char *argv[]) {
	std::string tip_frame;
	ros::init (argc, argv, "run_cbf_test");
	parse_arguments(argc, argv, tip_frame);

	rdf_loader::RDFLoader rdf("robot_description");
	const boost::shared_ptr<srdf::Model> &srdf = rdf.getSRDF();
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


	// run controller
	ros::Rate rate(50); // 50 hz update rate
	while (ros::ok()) {
		// do some nice target motion
		double t = ros::Time::now().toSec();
		double w = 2*M_PI / 10; // motion frequency for period 10s
		KDL::Frame tm(KDL::Rotation::EulerZYX(angles::from_degrees(-170),
		                                      angles::from_degrees(10),
		                                      angles::from_degrees(-30)),
		              KDL::Vector(0.2 * sin(w*t), 0.5 + 0.2 * cos(w*t), 0.1));
		target_vector.head(3) = Eigen::Map<Eigen::Vector3d>(tm.p.data);
		target_vector.tail(3) = Eigen::Map<Eigen::Vector3d>(tm.M.GetRot().data);
		// TODO: replace the target motion computation
		// by feedback from interactive marker
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
