#include <QtGui>

#include <rviz/panel.h>

class PluginPanel: public rviz::Panel
{
    Q_OBJECT

public:
    PluginPanel(QWidget* parent = 0);

protected:
    QVBoxLayout* _vbox;

    QPushButton* _button1;
    QPushButton* _button2;
};
