#include "QParamSlider.hpp"

    QParamSlider(QWidget* parent_, std::shared_ptr<rclcpp::Node> node_, std::string paramName, float min_, float max_, float defaultVal_, float precision_):
    QWidget(parent_),
    _node(node_),
    _param_client(std::make_shared<rclcpp::AsyncParametersClient>(_node)),
    _paramName(paramName),
    _min(min_),
    _max(max_),
    _defaultVal(defaultVal_),
    _precision(precision_)
{
    this->setupUi();
}

void QParamSlider::setupUi(void)
{
    _ui.setupUi(this);
    _ui.currentValue->setText(QString::number(_defaultVal));
    _ui.slider->setMinimum(_min);
    _ui.slider->setMaximum(_max);
    _ui.slider->setValue(_defaultVal);
    _ui.slider->setSingleStep(_precision);
}
