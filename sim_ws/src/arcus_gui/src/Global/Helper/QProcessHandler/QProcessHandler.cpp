#include "QProcessHandler.hpp"

#include <unistd.h>
#include <sys/types.h>

QProcessHandler::QProcessHandler(QWidget *parent_, std::string processName_, 
    std::string bashCmd_, bool forwardPrint_) : QWidget(parent_),
                                                _name(processName_),
                                                _bashCmd(bashCmd_),
                                                _forwardPrint(forwardPrint_)
{
    this->setupUi();
    this->connectSignals();
}

void QProcessHandler::onPushed(bool checked)
{
    if (checked)
    {
        if (_forwardPrint)
        {
            _process.start("setsid",
                QStringList() << "bash" << "-c"
                              << QString::fromStdString(_bashCmd));
        }
        else
        {
            _process.start("setsid",
                QStringList() << "xterm" << "-e"
                              << "bash" << "-c"
                              << QString::fromStdString(_bashCmd));
        }

        _ui.process_PB->setProperty("state", "success");
        _ui.process_PB->style()->unpolish(_ui.process_PB);
        _ui.process_PB->style()->polish(_ui.process_PB);
    }
    else
    {
        uint64_t pid = static_cast<uint64_t>(_process.processId());
        if (pid > 0)
        {
            ::kill(-pid, SIGTERM);
            _process.waitForFinished(PROCESS_TERM_TIMEOUT_MS);

            if (_process.state() != QProcess::NotRunning)
            {
                ::kill(-pid, SIGKILL);
                _process.waitForFinished(PROCESS_KILL_TIMEOUT_MS);
            }
        }

        _ui.process_PB->setProperty("state", "normal");
        _ui.process_PB->style()->unpolish(_ui.process_PB);
        _ui.process_PB->style()->polish(_ui.process_PB);
    }

    _ui.process_PB->style()->unpolish(_ui.process_PB);
    _ui.process_PB->style()->polish(_ui.process_PB);
}


void QProcessHandler::onStart()
{
    RCLCPP_INFO(rclcpp::get_logger("GUI"), "Process %s has started", _name.c_str());
}

void QProcessHandler::onStop()
{
    RCLCPP_INFO(rclcpp::get_logger("GUI"), "Process %s has stopped", _name.c_str());
}

void QProcessHandler::setupUi(void)
{
    _ui.setupUi(this);
    _ui.process_PB->setText(QString::fromStdString(_name));
    _ui.process_PB->setText(QString::fromStdString(_name));
}

void QProcessHandler::connectSignals(void)
{
    connect(_ui.process_PB, &QPushButton::toggled, this, &QProcessHandler::onPushed);
    connect(&_process, &QProcess::started, this, &QProcessHandler::onStart);
    connect(&_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &QProcessHandler::onStop);

    if (_forwardPrint)
    {
        connect(&_process, &QProcess::readyReadStandardOutput, [this]()
                { forwardPrints(_process); });
        connect(&_process, &QProcess::readyReadStandardError, [this]()
                { forwardPrints(_process); });
    }
}

void QProcessHandler::forwardPrints(QProcess &process_)
{
    std::cout << process_.readAllStandardOutput().toStdString();
    std::cerr << process_.readAllStandardError().toStdString();
}