#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include <QMainWindow>
#include <QShortcut>
#include <QFrame>
#include <QCloseEvent>
#include <QApplication>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "Global/Helper/QTopicSelector/QTopicSelector.hpp"

#include "QWidget/QArcusMaster/QArcusMaster.hpp"
#include "QWidget/QLocalNodes/QLocalNodes.hpp"
#include "rclcpp/rclcpp.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget = QWidget(this); // main widget that holds all other widgets, set as central widget of QMainWindow
    QGridLayout _gridLayout = QGridLayout(&_centralWidget); // grid layout to hold child widgets

    QArcusMaster _arcusMasterWidget;
    QWidget _exampleWidget1;
    QLocalNodesWidget _localNodesWidget;
    QWidget _exampleWidget3;
    QWidget _exampleWidget4;
    QWidget _exampleWidget5;
};

#endif  // MAIN_WINDOWS_HPP