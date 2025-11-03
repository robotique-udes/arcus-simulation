#include "waypoints_publisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointsPublisher>());
    rclcpp::shutdown();
    return 0;
}

WaypointsPublisher::WaypointsPublisher() : Node("waypoints_publisher")
{
    this->handleRosParam();
    this->loadWaypointsFromCSV();

    _waypointsPubTimer = this->create_wall_timer(std::chrono::milliseconds(PUBLISH_WAYPOINTS_DELAY_MS),
                                                 [this]()
                                                 {
                                                     this->CB_timerPublishWaypoints();
                                                 });

    _waypointsPublisher = this->create_publisher<nav_msgs::msg::Path>(WAYPOINTS_TOPIC, DEFAULT_QOS);
}

void WaypointsPublisher::handleRosParam(void)
{
    this->declare_parameter<std::string>("waypoints_file_path", DEFAULT_WAYPOINTS_FILE_PATH);

    _waypointsFilePath = this->get_parameter("waypoints_file_path").as_string();

    RCLCPP_INFO(this->get_logger(), "Waypoints file path: %s", _waypointsFilePath.c_str());
}

void WaypointsPublisher::loadWaypointsFromCSV(void)
{
    std::ifstream inputFile(_waypointsFilePath);

    if (!inputFile.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open specified file for waypoints : '%s'", _waypointsFilePath.c_str());
        return;
    }

    if (inputFile.peek() == std::ifstream::traits_type::eof())
    {
        RCLCPP_ERROR(this->get_logger(), "Specified file containing waypoints is empty : '%s'", _waypointsFilePath.c_str());
        return;
    }

    std::string line;

    while (std::getline(inputFile, line))
    {
        std::stringstream ss(line);

        std::string xPos;
        std::string yPos;

        std::getline(ss, xPos, ',');
        std::getline(ss, yPos, ',');

        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.pose.position.x = std::stod(xPos);
        poseStamped.pose.position.y = std::stod(yPos);
        poseStamped.pose.orientation.w = 1.0; // Neutral orientation

        poseStamped.header.frame_id = "map";
        poseStamped.header.stamp = this->now();

        _waypoints.push_back(poseStamped);
    }
    inputFile.close();
}

void WaypointsPublisher::CB_timerPublishWaypoints(void)
{
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.frame_id = "map";
    pathMsg.poses = _waypoints;

    _waypointsPublisher->publish(pathMsg);
}
