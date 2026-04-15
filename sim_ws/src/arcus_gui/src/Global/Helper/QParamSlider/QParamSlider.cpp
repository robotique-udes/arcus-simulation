#include "QParamSlider.hpp"

QParamSlider::QParamSlider(QWidget* parent_, std::shared_ptr<rclcpp::Node> node_):
    QWidget(parent_),
    _node(node_)
{
    this->setupUi();
}

void QParamSlider::setupUi(void)
{
    _ui.setupUi(this);
}
