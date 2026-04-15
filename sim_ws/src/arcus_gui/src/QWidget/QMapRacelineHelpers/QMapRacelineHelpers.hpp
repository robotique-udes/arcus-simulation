#ifndef Q_MAP_RACELINE_HELPERS_HPP
#define Q_MAP_RACELINE_HELPERS_HPP

#include "UI_MapRacelineHelpers.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"

class QMapRacelineHelpers : public QWidget
{
    Q_OBJECT

    static constexpr const char* MAP_CAR_TO_LOCAL_CMD = "source /opt/ros/humble/setup.bash && ros2 launch f1tenth_gym_ros visualize_launch.py";
    static constexpr const char* RACELINE_SCRIPT_CMD = "source /opt/ros/humble/setup.bash && ros2 launch drive_controller drive_controller.launch.py";
    static constexpr const char* WAYPOINTS_LOCAL_TO_CAR_CMD = "source /opt/ros/humble/setup.bash && ros2 launch drive_controller drive_controller.launch.py";

  public:
    QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:

  private:
    void setupUI(void);

    std::shared_ptr<rclcpp::Node> _node;

    QProcessHandler _mapCarToLocalProcess;
    QProcessHandler _racelineScriptProcess;
    QProcessHandler _waypointsLocalToCarProcess;

    Ui::MapRacelineHelpers _ui;
};

#endif  // Q_MAP_RACELINE_HELPERS_HPP
