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

#include <interactive_markers/interactive_marker_server.h>

namespace po = boost::program_options;
namespace vm = visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

vm::InteractiveMarkerFeedback marker_feedback;

void processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
	marker_feedback.pose.position.x = feedback.pose.position.x;
	marker_feedback.pose.position.y = feedback.pose.position.y;
	marker_feedback.pose.position.z = feedback.pose.position.z;
	
	marker_feedback.pose.orientation.w = feedback.pose.orientation.w;
	marker_feedback.pose.orientation.x = feedback.pose.orientation.x;
	marker_feedback.pose.orientation.y = feedback.pose.orientation.y;
	marker_feedback.pose.orientation.z = feedback.pose.orientation.z;
	
	marker_feedback.header.frame_id = feedback.header.frame_id;
	marker_feedback.header.stamp = feedback.header.stamp;
	
//TEMP -> pending clean up
	
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}

vm::Marker makeBox( vm::InteractiveMarker &msg )
{
  vm::Marker marker;

  marker.type = vm::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

vm::InteractiveMarkerControl& makeBoxControl( vm::InteractiveMarker &msg )
{
  vm::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void saveMarker( vm::InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

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

//////////////////////////

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3 &position, bool show_6dof )
{
  vm::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "6dof marker";
  int_marker.description = "6-DOF Marker Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  vm::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = vm::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}

//////////////////////////

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
	
	
	////////////////////////////////////////////////
	
	server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
	
	tf::Vector3 init_position = tf::Vector3(target.segment(0,3));
	tf::Quaternion init_orientation = tf::Quaternion(target.segment(3,4));
//TEMP -> check validity
	make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, init_position, true );
	
	
	////////////////////////////////////////////////


	// run controller
	ros::Rate rate(50); // 50 hz update rate
	while (ros::ok()) {
		
		
		KDL::Frame tm(KDL::Rotation::Quaternion(marker_feedback.pose.orientation),
		              KDL::Vector(marker_feedback.pose.position);
		target_vector.head(3) = Eigen::Map<Eigen::Vector3d>(tm.p.data);
		target_vector.tail(3) = Eigen::Map<Eigen::Vector3d>(tm.M.GetRot().data);

		target->set_reference(target_vector);

		// perform controller step
		controller->step();

		// fill + publish ros joint_state message
		update_message(js_msg, joints);
		jsp.publish(js_msg);

//
		// ?
		server.applyChanges();
//

		// process ros messages
		ros::spinOnce();
		rate.sleep();
	}
	
	server.reset();

	return EXIT_SUCCESS;
}
