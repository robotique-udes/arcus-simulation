#ifndef Q_PROCESS_HANDLER_HPP
#define Q_PROCESS_HANDLER_HPP

#include "UI_ProcessHandler.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include <QProcess>

class QProcessHandler : public QWidget
{
    Q_OBJECT

    /**
   * @brief Helper to hanlde a QProcess from a simple pushbutton. This can be used to launch and kill
   * ros2 node and packages.
   *
   * @param processName The name of the process displayed on the button
   * @param bashCmd The bash command used to launch the process
   * @param forwardPrint Whether to forward the process prints to the GUI terminal. If not, a terminal emulator will
   * be opened to display the process prints.
   */

  public:
    QProcessHandler(QWidget* parent_, std::string processName_, std::string bashCmd_, bool forwardPrint_ = true);

  private slots:
    void onPushed(void);
    void onStart(void);
    void onStop(void);

  private:
    void setupUi(void);
    void connectSignals(void);

    void static forwardPrints(QProcess& process_);

    std::string _name;
    std::string _bashCmd;
    bool _forwardPrint;

    Ui::ProcessHandler _ui;
    QProcess _process;
    bool _processIsOn = 0;
};

#endif  // Q_PROCESS_HANDLER_HPP
