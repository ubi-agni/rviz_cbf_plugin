#pragma once
#include "Controller.h"

namespace rviz_cbf_plugin
{

class RootController : public Controller
{
	Q_OBJECT
public:
	explicit RootController(rviz::Property *parent);

public:
	void showMarkers(const RegisteredMarkers &markers) const;
	void hideMarkers(const RegisteredMarkers &markers) const;
};

} // namespace rviz_cbf_plugin
