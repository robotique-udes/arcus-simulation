#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _exampleWidget(guiNode_, this),
    _topicSelector(this, "sensor_msgs/msg/LaserScan", "LaserScan", guiNode_),
    _visualizationProcess(this, "Visualization", VISUALIZATION_PROCESS_CMD, true),
    _controllerDriver(this, "Controller Driver", CONTROLLER_DRIVER, true),
    _speedWidget(guiNode_, this)
{
    this->setCentralWidget(&_centralWidget);

    _centralWidget.setLayout(&_gridLayout);
    _gridLayout.addWidget(&_visualizationProcess);
    _gridLayout.addWidget(&_controllerDriver);
<<<<<<< HEAD

    _gridLayout.addWidget(&_exampleWidget);
    _gridLayout.addWidget(&_topicSelector);
=======
    _gridLayout.addWidget(&_speedWidget);
>>>>>>> origin/EC_gui
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}