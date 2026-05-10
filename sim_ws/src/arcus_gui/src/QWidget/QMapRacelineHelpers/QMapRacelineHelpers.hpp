#ifndef Q_MAP_RACELINE_HELPERS_HPP
#define Q_MAP_RACELINE_HELPERS_HPP

#include "UI_MapRacelineHelpers.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"

class QMapRacelineHelpers : public QWidget
{
    Q_OBJECT

    static constexpr const char* RACELINE_HELPER_CMD = "python3 /sim_ws/src/scripts/map_preparation/prepare_map.py";

  public:
    QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:

  private:
    void setupUI(void);

    std::shared_ptr<rclcpp::Node> _node;

    QProcessHandler _racelineHelper;

    Ui::MapRacelineHelpers _ui;
};

#endif  // Q_MAP_RACELINE_HELPERS_HPP
