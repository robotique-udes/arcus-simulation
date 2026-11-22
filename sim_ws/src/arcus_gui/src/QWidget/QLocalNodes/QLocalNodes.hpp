#ifndef Q_LOCAL_NODES_WIDGET_HPP
#define Q_LOCAL_NODES_WIDGET_HPP

#include "UI_LocalNodes.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"
#include "Global/Helper/QCommandButton/QCommandButton.hpp"
#include <vector>

class QLocalNodesWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* VISUALIZATION_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch f1tenth_gym_ros visualize_launch.py";
    static constexpr const char* CONTROLLER_DRIVER = "source /opt/ros/humble/setup.bash && ros2 launch drive_controller drive_controller.launch.py";
    static constexpr const char* SOCKET_ADAPTER = "source /opt/ros/humble/setup.bash && ros2 launch socket_adapter socket.launch.py";
    static constexpr const char* SAVE_MAP_CMD = "source /opt/ros/humble/setup.bash && ros2 run slam_map_saver slam_map_saver";
    static constexpr const char* LOCALIZE_ROBOT_CMD = "source /opt/ros/humble/setup.bash && ros2 service call /reinitialize_global_localization std_srvs/srv/Empty";
    

  public:
    QLocalNodesWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:

  private:
    void setupUI(void);

    std::shared_ptr<rclcpp::Node> _node;

    QProcessHandler _visualizationProcess;
    QProcessHandler _controllerDriver;
    QProcessHandler _socketAdapterProcess;
    QCommandButton _saveMapProcess;
    QCommandButton _localizeRobotProcess;

    Ui::LocalNodes _ui;
};
 
#endif  
