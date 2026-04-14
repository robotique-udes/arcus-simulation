#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _arcusMasterWidget(guiNode_, this),
    _exampleWidget1(this),
    _localNodesWidget(guiNode_, this),
    _mapRacelineHelpers(guiNode_, this),
    _exampleWidget4(this),
    _exampleWidget5(this)
{
    
    this->setCentralWidget(&_centralWidget);
    _centralWidget.setLayout(&_gridLayout);

    _gridLayout.addWidget(&_arcusMasterWidget, 0, 0);
    _gridLayout.addWidget(&_exampleWidget1, 0, 1);
    _gridLayout.addWidget(&_localNodesWidget, 0, 2);
    _gridLayout.addWidget(&_mapRacelineHelpers, 1, 0);
    _gridLayout.addWidget(&_exampleWidget4, 1, 1);
    _gridLayout.addWidget(&_exampleWidget5, 1, 2);

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