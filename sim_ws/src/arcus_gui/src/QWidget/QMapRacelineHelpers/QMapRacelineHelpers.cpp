#include "QMapRacelineHelpers.hpp"

QMapRacelineHelpers::QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _racelineHelper(this, "Raceline Helpers", RACELINE_HELPER_CMD, true)
{
    _ui.setupUi(this);
    _ui.buttonsLayout->addWidget(&_racelineHelper);
}