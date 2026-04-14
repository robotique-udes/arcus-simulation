#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include <QMainWindow>
#include <QShortcut>
#include <QFrame>
#include <QCloseEvent>
#include <QApplication>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "Global/Helper/QTopicSelector/QTopicSelector.hpp"
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"
#include "QWidget/QCarInfoWidget/QCarInfoWidget.hpp"
#include "QWidget/QArcusMaster/QArcusMaster.hpp"
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

    QWidget _centralWidget = QWidget(this); // main widget that holds all other widgets, set as central widget of QMainWindow
    QGridLayout _gridLayout = QGridLayout(&_centralWidget); // grid layout to hold child widgets

    QArcusMaster _arcusMasterWidget;
    QCarInfoWidget _carInfoWidget;
    QWidget _exampleWidget2;
    QWidget _exampleWidget3;
    QWidget _exampleWidget4;
    QWidget _exampleWidget5;
};

#endif  // MAIN_WINDOWS_HPP