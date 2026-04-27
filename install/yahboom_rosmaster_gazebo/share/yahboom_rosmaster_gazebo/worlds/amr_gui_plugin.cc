#include "amr_gui_plugin.hh"
#include <gazebo/gazebo.hh>   // ★ 新增這行：確保 Gazebo 底層系統巨集被正確載入
#include <gazebo/gui/gui.hh>
#include <QVBoxLayout>
#include <QFrame>

using namespace gazebo;

namespace gazebo
{
  // 建構子實作
  AMRTelemetryPlugin::AMRTelemetryPlugin() : GUIPlugin()
  {
    this->setStyleSheet(
      "QFrame { background-color : rgba(50, 50, 50, 200); color : white; font-size: 14px; font-weight: bold; border-radius: 5px; }"
    );

    QVBoxLayout *mainLayout = new QVBoxLayout;
    
    this->posLabel = new QLabel("AMR Position: (0.0, 0.0)");
    this->velLabel = new QLabel("Velocity: 0.0 m/s");
    
    mainLayout->addWidget(this->posLabel);
    mainLayout->addWidget(this->velLabel);

    mainLayout->setContentsMargins(10, 10, 10, 10);
    this->setLayout(mainLayout);

    this->resize(250, 80);
    this->move(10, 10);
  }

  // 解構子實作
  AMRTelemetryPlugin::~AMRTelemetryPlugin()
  {
  }
} // namespace 結束

// ★ 關鍵修正：
// 1. 不要加 gazebo:: (因為上面已經 using namespace 了)
// 2. 絕對不可以加分號 (;)
GZ_REGISTER_GUIPLUGIN(AMRTelemetryPlugin)