#ifndef Q_PARAM_SLIDER_HPP
#define Q_PARAM_SLIDER_HPP

#include "UI_ParamSlider.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QParamSlider : public QWidget
{
    Q_OBJECT

  public:
    QParamSlider(QWidget* parent_, std::shared_ptr<rclcpp::Node> node_, std::string paramName, std::string remoteNodeName, float min_, float max_, float defaultVal_, float precision_);

  private slots:
    void onSliderMoved(int value);
    void onValueManuallyChanged(const QString& text);
    void onApplyClicked();

  private:
    void setupUi(void);
    void connectSignals(void);

    std::shared_ptr<rclcpp::Node> _node;
    std::shared_ptr<rclcpp::AsyncParametersClient> _param_client;

    std::string _paramName;
    std::string _remoteNodeName;
    float _min;
    float _max;
    float _defaultVal;
    float _precision;
    Ui::paramSlider _ui;
};

#endif  // Q_PARAM_SLIDER_HPP
