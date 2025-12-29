#ifndef Q_PROCESS_HANDLER_HPP
#define Q_PROCESS_HANDLER_HPP

#include "UI_ProcessHandler.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include <QProcess>
#include <QStyle>

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

    static constexpr uint16_t PROCESS_TERM_TIMEOUT_MS = 3000;
    static constexpr uint16_t PROCESS_KILL_TIMEOUT_MS = 1000;

  public:
    QProcessHandler(QWidget *parent_, std::string processName_, std::string bashCmd_, bool forwardPrint_ = true);

  private slots:
    void onPushed(bool checked);
    void onStart(void);
    void onStop(void);

  private:
    void setupUi(void);
    void connectSignals(void);

    void static forwardPrints(QProcess &process_);

    std::string _name;
    std::string _bashCmd;
    bool _forwardPrint;

    Ui::ProcessHandler _ui;
    QProcess _process;
};

#endif // Q_PROCESS_HANDLER_HPP
