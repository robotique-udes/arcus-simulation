#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _exampleWidget(guiNode_, this),
    _topicSelector(this, "sensor_msgs/msg/LaserScan", "LaserScan", guiNode_),
    _processHandler(this, "Simulation", "source /opt/ros/humble/setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py", false)
{
    this->setCentralWidget(&_centralWidget);

    _centralWidget.setLayout(&_gridLayout);
    _gridLayout.addWidget(&_exampleWidget);
    _gridLayout.addWidget(&_topicSelector);
    _gridLayout.addWidget(&_processHandler);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}