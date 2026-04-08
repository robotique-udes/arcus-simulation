#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _arcusMasterWidget(guiNode_, this),
    _carInfoWidget(guiNode_, this),
    _exampleWidget2(this),
    _exampleWidget3(this),
    _exampleWidget4(this),
    _exampleWidget5(this)
    //_visualizationProcess(this, "Visualization", VISUALIZATION_PROCESS_CMD, true)
    //_controllerDriver(this, "Controller Driver", CONTROLLER_DRIVER, true)
{
    
    this->setCentralWidget(&_centralWidget);
    _centralWidget.setLayout(&_gridLayout);

    _gridLayout.addWidget(&_arcusMasterWidget, 0, 0);
    _gridLayout.addWidget(&_carInfoWidget, 0, 1);
    _gridLayout.addWidget(&_exampleWidget2, 0, 2);
    _gridLayout.addWidget(&_exampleWidget3, 1, 0);
    _gridLayout.addWidget(&_exampleWidget4, 1, 1);
    _gridLayout.addWidget(&_exampleWidget5, 1, 2);
    //_gridLayout.addWidget(&_visualizationProcess, 1, 2);
    //_gridLayout.addWidget(&_controllerDriver, 2, 1);

    _gridLayout.setColumnStretch(0, 1);
    _gridLayout.setColumnStretch(1, 1);
    _gridLayout.setColumnStretch(2, 1);
    _gridLayout.setRowStretch(0, 1);
    _gridLayout.setRowStretch(1, 1);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}