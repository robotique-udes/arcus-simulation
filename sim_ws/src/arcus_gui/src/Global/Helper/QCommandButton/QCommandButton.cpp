#include "QCommandButton.hpp"

#include <rclcpp/rclcpp.hpp>

QCommandButton::QCommandButton(QWidget* parent_, std::string name_, std::string bashCmd_, bool forwardPrint_):
    QWidget(parent_),
    _name(name_),
    _bashCmd(bashCmd_),
    _forwardPrint(forwardPrint_)
{
    _ui.setupUi(this);
    _ui.process_PB->setCheckable(false);
    _ui.process_PB->setText(QString::fromStdString(_name));

    connect(_ui.process_PB, &QPushButton::clicked, this, &QCommandButton::onClicked);
}

void QCommandButton::onClicked(void)
{
    auto* process = new QProcess(this);

    connect(process, &QProcess::started, this, [this]() 
    {
        RCLCPP_INFO(rclcpp::get_logger("GUI"), "Command %s has started", _name.c_str());
    });
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
            [this, process](int, QProcess::ExitStatus) {
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "Command %s has finished", _name.c_str());
                process->deleteLater();
            });
    connect(process, &QProcess::errorOccurred, this, [this, process](QProcess::ProcessError error_) 
    {

        if (error_ == QProcess::FailedToStart)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Command %s failed to start", _name.c_str());
            process->deleteLater();
        }
    });

    const QString cmd = QString::fromStdString(_bashCmd);
    
    process->setProcessChannelMode(QProcess::ForwardedChannels);
    process->start("bash", QStringList() << "-c" << cmd);
    
  
}