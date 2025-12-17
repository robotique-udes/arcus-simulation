#ifndef Q_TOPIC_SELECTOR_HPP
#define Q_TOPIC_SELECTOR_HPP

#include "UI_TopicSelector.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QTopicSelector : public QWidget
{
    Q_OBJECT

    static constexpr const uint32_t TOPIC_SEARCH_FREQ_HZ = 1U;

    /**
   * @brief Helper to have a drop-down menu displaying available topic on given message type.
   * Then, user can poll the selected topic from the widget whenever needed with getSelectedTopic().
   *
   * @param msgType_ The target message type to filter topics (e.g., "nav_msgs/msg/Odometry")
   * @param name_ The name displayed on the selector (e.g., Position Topic)

   */

  public:
    QTopicSelector(QWidget* parent_, std::string msgType_, std::string name_, 
                   std::shared_ptr<rclcpp::Node> node_);
    std::string getSelectedTopic(void);

  signals:
    void newTopicFound(const std::string& topicName_);
    void removeTopic(const std::string& topicName_);

  private slots:
    void onNewTopicFound(const std::string& topicName_);
    void onRemoveTopic(const std::string& topicName_);

  private:
    void setupUi(void);
    void connectSignals(void);
    void initCallbacks(void);

    void CB_updateTopicSelection();

    std::string _targetMsgType;
    rclcpp::TimerBase::SharedPtr _searchTopicTimer;

    std::string _name;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::TopicSelector _ui;
};

#endif  // Q_TOPIC_SELECTOR_HPP
