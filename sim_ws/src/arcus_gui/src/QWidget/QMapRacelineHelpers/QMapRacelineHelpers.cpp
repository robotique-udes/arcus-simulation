#include "QMapRacelineHelpers.hpp"

QMapRacelineHelpers::QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _mapCarToLocalProcess(this, "Map (Car to Local)", MAP_CAR_TO_LOCAL_CMD, true),
    _racelineScriptProcess(this, "Raceline Script", RACELINE_SCRIPT_CMD, true),
    _waypointsLocalToCarProcess(this, "Waypoints (Local to Car)", WAYPOINTS_LOCAL_TO_CAR_CMD, true)
{
    _ui.setupUi(this);
    _ui.buttonsLayout->addWidget(&_mapCarToLocalProcess);
    _ui.buttonsLayout->addWidget(&_racelineScriptProcess);
    _ui.buttonsLayout->addWidget(&_waypointsLocalToCarProcess);
}