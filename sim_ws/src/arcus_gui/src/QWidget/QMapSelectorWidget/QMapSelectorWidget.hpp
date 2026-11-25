#ifndef Q_MAPSELECTORWIDGET_HPP
#define Q_MAPSELECTORWIDGET_HPP

#include "UI_MapSelector.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav2_msgs/srv/load_map.hpp"


#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include <QDir>
#include <QFileInfoList>
#include <QTimer>
#include <QStringList>
#include <QString>
#include <QFileDialog>
#include <QDebug>
#include <QPixmap>
#include <QResizeEvent>
#include <QImageReader>




class QMapSelectorWidget : public QWidget  
{
    Q_OBJECT


    const QString DEFAULT_MAPS_PATH = ("/sim_ws/src/arcus/slam_map_saver/slam_maps");
    const QStringList MAP_FILTERS = {"*.pgm"};
    static constexpr const size_t REFRESH_DELAY_MS = 100UL;

  public:

    QMapSelectorWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);
    QString currentMapPath(void) const;
    
  signals:
    
    void loadMapFinished(bool success_,const QString& message_);
    void mapChanged(const QString& map_path_);

    
  private:

    void onNewMapClicked(void);

    void resizeEvent(QResizeEvent* event_) override;

    void updatePreview(const QString& path_);

    void rescalePreview(void);    

    void browse(void);

    void onComboTextChanged(const QString& name_);

    void refreshMaps(void);

    void setupUI(void);

    void initSubscriber(void);

    QStringList listMaps(void) const;

    QPixmap _preview_pixmap;


    QTimer* _timer;

    std::shared_ptr<rclcpp::Node> _node;

    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr _load_map_client;

    Ui::MapSelectorWidget _ui;
};

#endif  // Q_MAPSELECTOR_WIDGET_HPP
