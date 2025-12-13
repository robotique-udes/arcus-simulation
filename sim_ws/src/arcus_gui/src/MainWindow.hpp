#ifndef MAIN_WINDOWS_HPP
#define MAIN_WINDOWS_HPP

#include <QMainWindow>
#include <QShortcut>
#include <QFrame>
#include <QCloseEvent>
#include <QApplication>
#include <QGridLayout>

#include "QWidget/QExampleWidget/QExampleWidget.hpp"
#include "rclcpp/rclcpp.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_);

  private:
    void closeEvent(QCloseEvent* event_) override;

    QWidget _centralWidget = QWidget(this);
    QGridLayout _gridLayout = QGridLayout(&_centralWidget);

    QExampleWidget _exampleWidget;
};

#endif  // MAIN_WINDOWS_HPP