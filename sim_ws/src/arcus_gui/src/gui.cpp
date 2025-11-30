#include "Global/Constant/StyleSheet.hpp"

#include "MainWindow.hpp"

#include <QApplication>
#include <QLabel>
#include <QWidget>
#include <QProcess>
#include <thread>

#include <rclcpp/rclcpp.hpp>

constexpr char WM_CLASS[] = "Arcus GUI";

int guiMain(int argc_, char* argv_[], std::shared_ptr<rclcpp::Node> guiNode_);
void displayWindows(MainWindow& mainWindow_);
void nodeThreadFunc(std::shared_ptr<rclcpp::Node> node);
void forwardPrints(QProcess& process_);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> guiNode = std::make_shared<rclcpp::Node>("gui_node");
    std::jthread rosThread(nodeThreadFunc, guiNode);

    int ret = guiMain(argc, argv, guiNode);

    rclcpp::shutdown();
    if (rosThread.joinable())
    {
        rosThread.join();
    }
    return ret;
}

/**
 * @brief Using a "2nd" main function to encapsulate all Qt elements in its own scope to make sure all destructors are called
 * after exiting the UI app independently of ros execution.
 *
 * @param argc_
 * @param argv_
 * @param guiNode_
 * @return int
 */
int guiMain(int argc_, char* argv_[], std::shared_ptr<rclcpp::Node> guiNode_)
{
    QApplication app(argc_, argv_);
    QApplication::setApplicationName(WM_CLASS);
    QApplication::setStyle("Fusion");
    app.setStyleSheet(QString(Constants::Style::STYLE_DARK_MODE) + QString(Constants::Style::STATUS_STYLE));

    MainWindow mainWindow(guiNode_);
    displayWindows(mainWindow);

    QProcess simProcess;
    // clang-format off
    // Example of how to forward prints from ros process with our node (we don't use it with the simProcess so we don't pollute the terminal)
    //QObject::connect(&simProcess, &QProcess::readyReadStandardOutput, [&simProcess](){ forwardPrints(simProcess); });
    //QObject::connect(&simProcess, &QProcess::readyReadStandardError, [&simProcess](){ forwardPrints(simProcess); });
    // clang-format on

    // Forward the sim log in a terminal emulator so we don't pollute the GUI terminal
    simProcess.start("xterm",
                     QStringList() << "-e"
                                   << "bash -c 'ros2 launch f1tenth_gym_ros gym_bridge_launch.py; exec bash'");

    int ret = QApplication::exec();

    simProcess.terminate();
    if (!simProcess.waitForFinished(3000))
    {
        simProcess.kill();
    }

    return ret;
}

void displayWindows(MainWindow& mainWindow_)
{
    QList<QScreen*> screens = QGuiApplication::screens();

    switch (screens.size())
    {
        case 0:
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Can't show GUI without screens");
            break;
        case 1:
        {
            QRect screenGeometry = screens[0]->geometry();
            int screenWidth = screenGeometry.width();
            int screenHeight = screenGeometry.height();

            mainWindow_.setGeometry(screenGeometry.x(), screenGeometry.y(), screenWidth / 2, screenHeight);
            mainWindow_.show();
            break;
        }

        default:
            mainWindow_.setGeometry(screens[0]->geometry());
            mainWindow_.showMaximized();
            break;
    }
}

void nodeThreadFunc(std::shared_ptr<rclcpp::Node> node_)
{
    rclcpp::executors::SingleThreadedExecutor rosExecutor;
    rosExecutor.add_node(node_);
    rosExecutor.spin();

    rosExecutor.remove_node(node_);
}

void forwardPrints(QProcess& process_)
{
    std::cout << process_.readAllStandardOutput().toStdString();
    std::cerr << process_.readAllStandardError().toStdString();
}

#ifndef __INTELLISENSE__
#include "gui.moc"
#endif  // __INTELLISENSE__