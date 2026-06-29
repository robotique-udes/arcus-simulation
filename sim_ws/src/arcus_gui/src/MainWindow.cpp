#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _arcusMasterWidget(guiNode_, this),
    _carInfoWidget(guiNode_, this),
    _localNodesWidget(guiNode_, this),
    _mapRacelineHelpers(guiNode_, this),
    _purePursuitWidget(guiNode_, this),
    _gapFollowWidget(guiNode_, this)
{
    
    this->setCentralWidget(&_centralWidget);
    _centralWidget.setLayout(&_gridLayout);

    _gridLayout.addWidget(&_arcusMasterWidget, 0, 0);
    _gridLayout.addWidget(&_carInfoWidget, 0, 1);
    _gridLayout.addWidget(&_localNodesWidget, 0, 2);
    _gridLayout.addWidget(&_mapRacelineHelpers, 1, 0);
    _gridLayout.addWidget(&_purePursuitWidget, 1, 1);
    _gridLayout.addWidget(&_gapFollowWidget, 1, 2);

    _gridLayout.setColumnStretch(0, 1);
    _gridLayout.setColumnStretch(1, 1);
    _gridLayout.setColumnStretch(2, 1);
    _gridLayout.setRowStretch(0, 1);
    _gridLayout.setRowStretch(1, 1);




    _globalSaveButton = new QPushButton("Save All Tuning Configurations to Disk (YAML)", this);
    _globalSaveButton->setMinimumHeight(45);
    _gridLayout.addWidget(_globalSaveButton, 2, 0, 1, 3); 

    _gridLayout.setColumnStretch(0, 1);
    _gridLayout.setColumnStretch(1, 1);
    _gridLayout.setColumnStretch(2, 1);
    _gridLayout.setRowStretch(0, 10);
    _gridLayout.setRowStretch(1, 10);
    _gridLayout.setRowStretch(2, 1);

    _paramSaverClient = _node->create_client<std_srvs::srv::Trigger>("/arcus/save_parameters");

    connect(_globalSaveButton, &QPushButton::clicked, this, &MainWindow::onGlobalSaveClicked);
}

void MainWindow::onGlobalSaveClicked(void)
{
    if (!_paramSaverClient->wait_for_service(std::chrono::milliseconds(500)))
    {
        RCLCPP_ERROR(_node->get_logger(), "Global parameter saver node is not running!");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    RCLCPP_INFO(_node->get_logger(), "Sending call to save all active parameters to disk...");
    
    _paramSaverClient->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(_node->get_logger(), "SUCCESS: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(_node->get_logger(), "FAILURE: %s", response->message.c_str());
            }
        });
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}