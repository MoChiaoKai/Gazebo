#ifndef AMR_GUI_PLUGIN_HH
#define AMR_GUI_PLUGIN_HH

#include <gazebo/gui/GuiPlugin.hh>
#include <QLabel>

namespace gazebo
{
  class AMRTelemetryPlugin : public GUIPlugin
  {
    Q_OBJECT

    public: AMRTelemetryPlugin();
    public: virtual ~AMRTelemetryPlugin();

    private: QLabel *posLabel;
    private: QLabel *velLabel;
  };
}

#endif