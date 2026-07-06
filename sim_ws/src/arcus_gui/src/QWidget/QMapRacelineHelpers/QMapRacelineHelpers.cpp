#include "QMapRacelineHelpers.hpp"

QMapRacelineHelpers::QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _racelineHelper(this, "Raceline Helpers", RACELINE_HELPER_CMD, true),
    _ttcDecayRateSlider(parent_, node_, TTC_DECAY_RATE_PARAM_NAME, PURE_PURSUIT_NODE_NAME, TTC_DECAY_RATE_MIN, TTC_DECAY_RATE_MAX, TTC_DECAY_RATE_DEFAULT, TTC_DECAY_RATE_PRECISION),
    _maxRiskSlider(parent_, node_, MAX_RISK_PARAM_NAME, MASTER_NODE_NAME, MAX_RISK_MIN, MAX_RISK_MAX, MAX_RISK_DEFAULT, MAX_RISK_PRECISION)
{
    _ui.setupUi(this);
    _ui.buttonsLayout->addWidget(&_racelineHelper);
    _ui.buttonsLayout->addWidget(&_ttcDecayRateSlider);
    _ui.buttonsLayout->addWidget(&_maxRiskSlider);

    this->connectSignals();
}

void QMapRacelineHelpers::connectSignals(void)
{
    connect(_ui.applyAllPB, &QPushButton::clicked, this, &QMapRacelineHelpers::onApplyAllClicked);
}

void QMapRacelineHelpers::onApplyAllClicked(void)
{
    const auto sliders = this->findChildren<QParamSlider*>();
    for (auto* slider : sliders)
    {
        slider->onApplyClicked();
    }
}