#include "MainWindow.hpp"

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    QMainWindow(nullptr),
    _node(guiNode_),
    _arcusMasterWidget(guiNode_, this),
    _carInfoWidget(guiNode_, this),
    _localNodesWidget(guiNode_, this),
    _mapRacelineHelpers(guiNode_, this),
    _purePursuitWidget(guiNode_, this),
    _gapFollowWidget(guiNode_, this),
    _paramSaverWidget(guiNode_, this)
{
    
    this->setCentralWidget(_centralWidget);
    _centralWidget->setLayout(&_gridLayout);
    _gridLayout.setContentsMargins(10, 10, 10, 10);
    _gridLayout.setHorizontalSpacing(10);
    _gridLayout.setVerticalSpacing(10);

    _gridLayout.addWidget(&_arcusMasterWidget, 0, 0);
    _gridLayout.addWidget(&_carInfoWidget, 0, 1);
    _gridLayout.addWidget(&_localNodesWidget, 0, 2);
    _gridLayout.addWidget(&_mapRacelineHelpers, 1, 0);
    _gridLayout.addWidget(&_purePursuitWidget, 1, 1);
    _gridLayout.addWidget(&_gapFollowWidget, 1, 2);
    _paramSaverWidget.setFixedHeight(58);
    _gridLayout.addWidget(&_paramSaverWidget, 2, 0, 1, 3);

    _gridLayout.setColumnStretch(0, 1);
    _gridLayout.setColumnStretch(1, 1);
    _gridLayout.setColumnStretch(2, 1);
    _gridLayout.setRowStretch(0, 1);
    _gridLayout.setRowStretch(1, 1);
    _gridLayout.setRowStretch(2, 0);

    QWidget* panels[] = {
        &_arcusMasterWidget, &_carInfoWidget, &_localNodesWidget,
        &_mapRacelineHelpers, &_purePursuitWidget, &_gapFollowWidget};
    for (QWidget* panel : panels)
    {
        panel->setMinimumSize(0, 0);
        panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    connect(&_paramSaverWidget, &QParamSaverWidget::profileReloaded, &_gapFollowWidget, &QGapFollowWidget::refreshSliderValues);
    connect(&_paramSaverWidget, &QParamSaverWidget::profileReloaded, &_mapRacelineHelpers, &QMapRacelineHelpers::refreshSliderValues);
    connect(&_paramSaverWidget, &QParamSaverWidget::profileReloaded, &_purePursuitWidget, &QPurePursuitWidget::refreshSliderValues);
}

void MainWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}