#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _exampleWidget(guiNode_, this),
    _topicSelector(this, "sensor_msgs/msg/LaserScan", "LaserScan", guiNode_),
    _simuProcess(this, "Simulation", SIM_PROCESS_CMD, false),
    _visualizationProcess(this, "Visualization", VISUALIZATION_PROCESS_CMD, true),
    _gapFollowProcess(this, "Gap Follow", GAP_FOLLOW_PROCESS_CMD, true),
    _purePusuitProcess(this, "Pure Pursuit", PURE_PURSUIT_PROCESS_CMD, true),
    _mapSaverProcess(this, "Map Saver", MAP_SAVER_PROCESS_CMD, true)
{
    this->setCentralWidget(&_centralWidget);

    _centralWidget.setLayout(&_gridLayout);
    _gridLayout.addWidget(&_exampleWidget);
    _gridLayout.addWidget(&_topicSelector);
    _gridLayout.addWidget(&_simuProcess);
    _gridLayout.addWidget(&_visualizationProcess);
    _gridLayout.addWidget(&_gapFollowProcess);
    _gridLayout.addWidget(&_purePusuitProcess);
    _gridLayout.addWidget(&_mapSaverProcess);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}