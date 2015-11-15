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
	friend class RootController;
public:
	typedef std::map<std::string, unsigned int> RegisteredMarkers;

	explicit Controller(const QString &name, const Controller &parent);

	const Controller* getParent() const {return parent_;}
	const RootController& getRoot() const {return root_;}

public Q_SLOTS:
	void showMarkers(bool bShow = true) const;
	void hideMarkers(bool bHide = true) const;

private:
	explicit Controller(const QString &name, rviz::Property *parent, const RootController &root);

private:
	Controller     const *parent_;
	const RootController &root_;
	RegisteredMarkers     markers_;
};

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
