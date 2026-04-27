#include "QLocalNodes.hpp"

QLocalNodesWidget::QLocalNodesWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _visualizationProcess(this, "Visualization", VISUALIZATION_PROCESS_CMD, true),
    _controllerDriver(this, "Controller Driver", CONTROLLER_DRIVER, true)
{
    _ui.setupUi(this);
    _ui.buttonsLayout->addWidget(&_visualizationProcess);
    _ui.buttonsLayout->addWidget(&_controllerDriver);
}