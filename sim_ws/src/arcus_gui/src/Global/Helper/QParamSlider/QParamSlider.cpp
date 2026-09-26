#include "QParamSlider.hpp"

QParamSlider::QParamSlider(QWidget* parent_, std::shared_ptr<rclcpp::Node> node_, std::string paramName, std::string remoteNodeName, float min_, float max_, float defaultVal_, float precision_):
    QWidget(parent_),
    _node(node_),
    _param_client(std::make_shared<rclcpp::AsyncParametersClient>(_node, remoteNodeName)),
    _paramName(paramName),
    _remoteNodeName(remoteNodeName),
    _min(min_),
    _max(max_),
    _defaultVal(defaultVal_),
    _precision(precision_)
{
    this->setupUi();
    this->connectSignals();
}

void QParamSlider::setupUi(void)
{
    _ui.setupUi(this);
    _ui.title->setText(QString::fromStdString(_paramName));
    _ui.currentValue->setText(QString::number(_defaultVal));

    _ui.slider->setMinimum(static_cast<int>(_min / _precision));
    _ui.slider->setMaximum(static_cast<int>(_max / _precision));
    _ui.slider->setValue(static_cast<int>(_defaultVal / _precision));
    _ui.slider->setSingleStep(1);
}

void QParamSlider::connectSignals(void)
{
    connect(_ui.slider, &QSlider::valueChanged, this, &QParamSlider::onSliderMoved);
    connect(_ui.currentValue, &QLineEdit::textChanged, this, &QParamSlider::onValueManuallyChanged);
    connect(_ui.applyPB, &QPushButton::clicked, this, &QParamSlider::onApplyClicked);
}

void QParamSlider::onSliderMoved(int value) 
{
    float realVal = value * _precision;

    _ui.currentValue->blockSignals(true);
    _ui.currentValue->setText(QString::number(realVal));
    _ui.currentValue->blockSignals(false);
}

void QParamSlider::onValueManuallyChanged(const QString& text) {

    bool isValidFloat;
    float val = text.toFloat(&isValidFloat);

    if (!isValidFloat) return;

    val = std::clamp(val, _min, _max);

    _ui.slider->blockSignals(true);
    _ui.slider->setValue(static_cast<int>(val / _precision));
    _ui.slider->blockSignals(false);
}

void QParamSlider::onApplyClicked() {

    float val = _ui.currentValue->text().toFloat();

    auto param = rclcpp::Parameter(_paramName, static_cast<double>(val));

    _param_client->set_parameters(
        {param},
        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
            auto results = future.get();
            if (!results.empty() && results[0].successful) {
                RCLCPP_INFO(_node->get_logger(), "Parameter '%s' set successfully", _paramName.c_str());
            } else {
                RCLCPP_WARN(_node->get_logger(), "Failed to set parameter '%s': %s",
                    _paramName.c_str(), results[0].reason.c_str());
            }
        }
    );
}

void QParamSlider::updateValue(float value)
{
    value = std::clamp(value, _min, _max);

    _ui.slider->blockSignals(true);
    _ui.currentValue->blockSignals(true);

    _ui.currentValue->setText(QString::number(value));
    _ui.slider->setValue(static_cast<int>(value / _precision));

    _ui.currentValue->blockSignals(false);
    _ui.slider->blockSignals(false);
}