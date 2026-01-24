#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include <QMainWindow>
#include <QShortcut>
#include <QFrame>
#include <QCloseEvent>
#include <QApplication>
#include <QGridLayout>
#include "QWidget/QExampleWidget/QExampleWidget.hpp"
#include "Global/Helper/QTopicSelector/QTopicSelector.hpp"
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"
#include "rclcpp/rclcpp.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

    static constexpr const char* SIM_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py";
    static constexpr const char* VISUALIZATION_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch visualization visualization.launch.py";
    static constexpr const char* GAP_FOLLOW_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch gap_follow gap_follow.launch.py";
    static constexpr const char* PURE_PURSUIT_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch pure_pursuit pure_pursuit.launch.py";
    static constexpr const char* MAP_SAVER_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch map_saver map_saver.launch.py";

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget = QWidget(this);
    QGridLayout _gridLayout = QGridLayout(&_centralWidget);

    QExampleWidget _exampleWidget;
    QTopicSelector _topicSelector;

    QProcessHandler _simuProcess;
    QProcessHandler _visualizationProcess;
    QProcessHandler _gapFollowProcess;
    QProcessHandler _purePusuitProcess;
    QProcessHandler _mapSaverProcess;



};

#endif  // MAIN_WINDOWS_HPP