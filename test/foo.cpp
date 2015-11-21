#include "ControllerDisplay.h"
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <QApplication>

using namespace rviz_cbf_plugin;

int main(int argc, char *argv[]) {
	ros::init (argc, argv, "foo");
	QApplication app(argc, argv);

	rviz::RenderPanel render_panel_;
	rviz::VisualizationManager manager_(&render_panel_);
	render_panel_.initialize(manager_.getSceneManager(), &manager_);
	manager_.initialize();
	manager_.startUpdate();

	for (int i = 0; i < 3; ++i) {
		auto foo = new ControllerDisplay();
		manager_.addDisplay(foo, true);
		delete foo;
	}
}
