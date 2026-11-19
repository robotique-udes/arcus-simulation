#ifndef Q_COMMAND_BUTTON_HPP
#define Q_COMMAND_BUTTON_HPP

#include "UI_ProcessHandler.h"

#include <QtWidgets/QWidget>
#include <QProcess>
#include <string>

class QCommandButton : public QWidget
{
    Q_OBJECT

  public:
    QCommandButton(QWidget* parent_, std::string name_, std::string bashCmd_, bool forwardPrint_ = true);

  private slots:
    void onClicked(void);

  private:
    std::string _name;
    std::string _bashCmd;
    bool _forwardPrint;

    Ui::ProcessHandler _ui;
};

#endif  