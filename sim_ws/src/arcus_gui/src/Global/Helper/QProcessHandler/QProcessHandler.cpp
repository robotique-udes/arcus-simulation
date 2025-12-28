#include "QProcessHandler.hpp"

QProcessHandler::QProcessHandler(QWidget* parent_, std::string processName_,std::string bashCmd_, bool forwardPrint_):
    QWidget(parent_),
    _name(processName_),
    _bashCmd(bashCmd_),
    _forwardPrint(forwardPrint_)
{
    this->setupUi();
    this->connectSignals();
}

void QProcessHandler::onPushed()
{
    if (!_processIsOn)
    {
        if (_forwardPrint)
        {
            _process.start("bash", QStringList() << "-c" << QString::fromStdString(_bashCmd));
        }
        else
        {
            _process.start("xterm",
                     QStringList() << "-e"
                                   << "bash -c" + QString::fromStdString(_bashCmd));
        }

        _processIsOn = true;
    }
    else
    {
        _process.kill();
        _processIsOn = false;
    }
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
}

void QProcessHandler::connectSignals(void)
{
    connect(_ui.process_PB, &QPushButton::clicked, this, &QProcessHandler::onPushed);
    QObject::connect(&_process, &QProcess::readyReadStandardOutput, [this](){ forwardPrints(_process); });
    QObject::connect(&_process, &QProcess::readyReadStandardError, [this](){ forwardPrints(_process); });
}

void QProcessHandler::forwardPrints(QProcess& process_)
{
    std::cout << process_.readAllStandardOutput().toStdString();
    std::cerr << process_.readAllStandardError().toStdString();
}
