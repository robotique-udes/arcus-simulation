#include "QTopicSelector.hpp"

QTopicSelector::QTopicSelector(QWidget* parent_, std::string msgType_, 
                               std:: string name_, std::shared_ptr<rclcpp::Node> node_):
    QWidget(parent_),
    _targetMsgType(msgType_),
    _name(name_),
    _node(node_)
{
    this->setupUi();
    this->connectSignals();
    this->initCallbacks();
}

std::string QTopicSelector::getSelectedTopic(void)
{
    return _ui.selector->currentText().toStdString();
}

void QTopicSelector::onNewTopicFound(const std::string& topicName_)
{
    _ui.selector->addItem(QString::fromStdString(topicName_));
}

void QTopicSelector::onRemoveTopic(const std::string& topicName_)
{
    for (int i = 0; i < _ui.selector->count(); i++)
    {
        if (_ui.selector->itemText(i) == QString::fromStdString(topicName_))
        {
            _ui.selector->removeItem(i);
            break;
        }
    }
}

void QTopicSelector::setupUi(void)
{
    _ui.setupUi(this);
    _ui.topic->setText(QString::fromStdString(_name + ": "));
}

void QTopicSelector::connectSignals(void)
{
    connect(this, &QTopicSelector::newTopicFound, this, &QTopicSelector::onNewTopicFound);
    connect(this, &QTopicSelector::removeTopic, this, &QTopicSelector::onRemoveTopic);
}

void QTopicSelector::initCallbacks(void)
{
    _searchTopicTimer = _node->create_wall_timer(std::chrono::milliseconds(1000/TOPIC_SEARCH_FREQ_HZ),
                                            [this](void)
                                            {
                                                this->CB_updateTopicSelection();
                                            });

}

void QTopicSelector::CB_updateTopicSelection(void)
{
    std::vector<std::string> currentTopicSelection;
    for (int i = 0; i < _ui.selector->count(); i++)
    {
        currentTopicSelection.push_back(_ui.selector->itemText(i).toStdString());    
    }

    std::map<std::string, std::vector<std::string>> topicAndTypes = _node->get_topic_names_and_types();

    std::vector<std::string> activeTopics;
    for (const auto& pair : topicAndTypes) {
        
        const auto& topicName = pair.first;
        const auto& msgType = pair.second;

        // Check for matching message type in active topics
        if (std::find(msgType.begin(), msgType.end(), _targetMsgType) != msgType.end()) 
        {
            // Check if found matching topic is already in the selection list
            if (std::find(currentTopicSelection.begin(), currentTopicSelection.end(), topicName) == currentTopicSelection.end()) 
            {
                currentTopicSelection.push_back(topicName);
                emit newTopicFound(topicName);
            }
        }
        activeTopics.push_back(topicName);
    }

    // Remove topics that are no longer active
    std::unordered_set<std::string> activeSet(activeTopics.begin(), activeTopics.end());

    for (const auto& topicName : currentTopicSelection) 
    {
        if (activeSet.find(topicName) == activeSet.end())
        {
            emit removeTopic(topicName);
        }
    }
}
