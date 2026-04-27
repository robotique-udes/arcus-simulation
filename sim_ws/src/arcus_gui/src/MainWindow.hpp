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
#include "Global/Helper/QProcessHandler/QProcessHandler.hpp"
#include "QWidget/QCarInfoWidget/QCarInfoWidget.hpp"

#include "QWidget/QArcusMaster/QArcusMaster.hpp"
#include "QWidget/QLocalNodes/QLocalNodes.hpp"
#include "QWidget/QPurePursuitWidget/QPurePursuit.hpp"
#include "QWidget/QMapRacelineHelpers/QMapRacelineHelpers.hpp"
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
    QCarInfoWidget _carInfoWidget;
    QLocalNodesWidget _localNodesWidget;
    QMapRacelineHelpers _mapRacelineHelpers;
    QPurePursuitWidget _purePursuitWidget;
    QWidget _exampleWidget5;
};

#endif  // MAIN_WINDOWS_HPP