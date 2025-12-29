#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _exampleWidget(guiNode_, this),
    _topicSelector(this, "sensor_msgs/msg/LaserScan", "LaserScan", guiNode_),
    _simuProcess(this, "Simulation", "source /opt/ros/humble/setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py", false),
    _visualizationProcess(this, "Visualization", "source /opt/ros/humble/setup.bash && ros2 launch visualization visualization.launch.py", true),
    _gapFollowProcess(this, "Gap Follow", "source /opt/ros/humble/setup.bash && ros2 launch gap_follow gap_follow.launch.py", true),
    _purePusuitProcess(this, "Pure Pursuit", "source /opt/ros/humble/setup.bash && ros2 launch pure_pursuit pure_pursuit.launch.py", true)


{
    this->setCentralWidget(&_centralWidget);

    _centralWidget.setLayout(&_gridLayout);
    _gridLayout.addWidget(&_exampleWidget);
    _gridLayout.addWidget(&_topicSelector);
    _gridLayout.addWidget(&_simuProcess);
    _gridLayout.addWidget(&_visualizationProcess);
    _gridLayout.addWidget(&_gapFollowProcess);
    _gridLayout.addWidget(&_purePusuitProcess);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}