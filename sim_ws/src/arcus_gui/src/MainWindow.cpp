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

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(&_visualizationProcess);
    buttonLayout->addWidget(&_controllerDriver);

    QHBoxLayout* bottomLayout = new QHBoxLayout();
    bottomLayout->addWidget(&_exampleWidget);
    bottomLayout->addWidget(&_topicSelector);

    // Add to main vertical layout
    _mainLayout.addLayout(buttonLayout);
    _mainLayout.addWidget(&_speedWidget, 1); // Give stretch to make it bigger
    _mainLayout.addLayout(bottomLayout);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}