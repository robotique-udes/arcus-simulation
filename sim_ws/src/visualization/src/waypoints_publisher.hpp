#ifndef WAYPOINTS_PUBLISHER_HPP
#define WAYPOINTS_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include <fstream>
#include <string>
#include <sstream>
#include <vector>

class WaypointsPublisher : public rclcpp::Node
{
    static constexpr const char* WAYPOINTS_TOPIC = "/waypoints";
    static constexpr const char* DEFAULT_WAYPOINTS_FILE_PATH = "/sim_ws/src/arcus/resources/waypoints/waypoints.csv";
    static constexpr uint32_t PUBLISH_WAYPOINTS_DELAY_MS = 1'000u;
    static constexpr uint32_t DEFAULT_QOS = 1;

  public:
    WaypointsPublisher();

  private:
    void handleRosParam(void);
    void loadWaypointsFromCSV(void);
    void CB_timerPublishWaypoints(void);

    std::string _waypointsFilePath = DEFAULT_WAYPOINTS_FILE_PATH;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _waypointsPublisher;
    rclcpp::TimerBase::SharedPtr _waypointsPubTimer;
    std::vector<geometry_msgs::msg::PoseStamped> _waypoints;
};

#endif  // WAYPOINTS_PUBLISHER_HPP