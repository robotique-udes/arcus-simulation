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

    static constexpr const char* VISUALIZATION_PROCESS_CMD = "source /opt/ros/humble/setup.bash && ros2 launch f1tenth_gym_ros visualize_launch.py";
    static constexpr const char* CONTROLLER_DRIVER = "source /opt/ros/humble/setup.bash && ros2 launch drive_controller drive_controller.launch.py";

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget = QWidget(this);
    QGridLayout _gridLayout = QGridLayout(&_centralWidget);

    QExampleWidget _exampleWidget;
    QTopicSelector _topicSelector;

    QProcessHandler _visualizationProcess;
    QProcessHandler _controllerDriver;
};

#endif  // MAIN_WINDOWS_HPP