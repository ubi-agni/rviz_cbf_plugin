#include <pluginlib/class_list_macros.h>

#include "plugin_panel.h"

PluginPanel::PluginPanel(QWidget* parent):
    rviz::Panel(parent)
{
    _vbox = new QVBoxLayout();

    _button1 = new QPushButton(tr("Button 1"));
    _button2 = new QPushButton(tr("Button 2"));

    _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    _vbox->addWidget(_button1);
    _vbox->addWidget(_button2);

    setLayout(_vbox);
}

PLUGINLIB_EXPORT_CLASS(PluginPanel, rviz::Panel)
