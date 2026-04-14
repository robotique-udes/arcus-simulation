#include "QPurePursuit.hpp"

QPurePursuitWidget::QPurePursuitWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);
}