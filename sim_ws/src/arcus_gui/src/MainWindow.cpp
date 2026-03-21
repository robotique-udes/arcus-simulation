#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _exampleWidget(guiNode_, this),
    _topicSelector(this, "sensor_msgs/msg/LaserScan", "LaserScan", guiNode_),
    _visualizationProcess(this, "Visualization", VISUALIZATION_PROCESS_CMD, true),
    _controllerDriver(this, "Controller Driver", CONTROLLER_DRIVER, true)
{
    this->setCentralWidget(&_centralWidget);

    _centralWidget.setLayout(&_gridLayout);
    _gridLayout.addWidget(&_visualizationProcess);
    _gridLayout.addWidget(&_controllerDriver);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}