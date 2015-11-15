#pragma once
#include "rviz/properties/property.h"
#include <map>
#include <string>

namespace rviz_cbf_plugin
{

class RootController;
class Controller : public rviz::Property
{
	Q_OBJECT
public:
	typedef std::map<std::string, unsigned int> RegisteredMarkers;

	explicit Controller(const QString &name, const Controller *parent);

public Q_SLOTS:
	void showMarkers(bool bShow = true) const;
	void hideMarkers(bool bHide = true) const;

protected:
	const Controller     *parent_;
	const RootController *root_;
	RegisteredMarkers     markers_;
};

} // namespace rviz_cbf_plugin
