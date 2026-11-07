#ifndef Q_MAP_RACELINE_HELPERS_HPP
#define Q_MAP_RACELINE_HELPERS_HPP

#include "UI_MapRacelineHelpers.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"
#include "Global/Helper/QParamSlider/QParamSlider.hpp"

class QMapRacelineHelpers : public QWidget
{
    Q_OBJECT

    static constexpr const char* PURE_PURSUIT_NODE_NAME = "/arcus/pure_pursuit";
    static constexpr const char* MASTER_NODE_NAME = "/arcus/master_node";

    static constexpr const char* TTC_DECAY_RATE_PARAM_NAME = "ttc_decay_rate";
    static constexpr const char* MAX_RISK_PARAM_NAME = "max_accepted_risk";

    static constexpr float TTC_DECAY_RATE_DEFAULT = 1.4;
    static constexpr float MAX_RISK_DEFAULT = 70;

    static constexpr float TTC_DECAY_RATE_MIN = 0.0;
    static constexpr float MAX_RISK_MIN = 0.0;

    static constexpr float TTC_DECAY_RATE_MAX = 3;
    static constexpr float MAX_RISK_MAX = 100;

    static constexpr float TTC_DECAY_RATE_PRECISION = 0.05;
    static constexpr float MAX_RISK_PRECISION = 0.05;

    static constexpr const char* RACELINE_HELPER_CMD = "python3 /sim_ws/src/scripts/map_preparation/prepare_map.py";

  public:
    QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

    void refreshSliderValues(void);
  signals:

  private:
    void setupUI(void);
    void connectSignals(void);
    void onApplyAllClicked(void);

    std::shared_ptr<rclcpp::Node> _node;

    QProcessHandler _racelineHelper;

    QParamSlider _ttcDecayRateSlider;
    QParamSlider _maxRiskSlider;

    std::map<std::string, rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr> _param_clients;

    Ui::MapRacelineHelpers _ui;
};

#endif  // Q_MAP_RACELINE_HELPERS_HPP
