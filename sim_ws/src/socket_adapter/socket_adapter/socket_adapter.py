import rclpy
from rclpy.node import Node
import foxglove_ros_client as humble_ros

# ==================== CONFIGURATION ====================
ROBOT_IP = "arcus-jetson.local"  # Replace with physical robot IP
FOXGLOVE_PORT = 8765

# IMPORT YOUR CUSTOM MESSAGES HERE
# (Assuming your package name is my_custom_interfaces)
from my_custom_interfaces.msg import CustomCommand, CustomStatus

# Topics leaving Docker -> heading to Physical Robot
TOPICS_TO_ROBOT = [
    ('/robot/command', 'my_custom_interfaces/msg/CustomCommand', CustomCommand)
]

# Topics leaving Robot -> arriving into Docker
TOPICS_FROM_ROBOT = [
    ('/robot/status', 'my_custom_interfaces/msg/CustomStatus', CustomStatus)
]
# ========================================================

class BidirectionalFoxgloveAdapter(Node):
    def __init__(self):
        super().__init__('foxglove_bidirectional_adapter')
        
        # Connect to Robot Foxglove Bridge Server
        self.get_logger().info(f"Connecting to ws://{ROBOT_IP}:{FOXGLOVE_PORT}...")
        self.ws_client = humble_ros.Ros(ROBOT_IP, FOXGLOVE_PORT)
        self.ws_client.run()
        self.get_logger().info("Established WebSocket connection.")

        # Outbound Traffic configuration
        self.active_humble_publishers = {}
        for topic_name, msg_type_str, msg_class in TOPICS_TO_ROBOT:
            humble_pub = humble_ros.Topic(self.ws_client, topic_name, msg_type_str)
            humble_pub.advertise()
            self.active_humble_publishers[topic_name] = humble_pub
            
            self.create_subscription(
                msg_class, 
                topic_name, 
                self.make_outbound_callback(topic_name), 
                10
            )
            self.get_logger().info(f"Bridged Outbound: Local DDS {topic_name} -> Robot WebSocket")

        # Inbound Traffic configuration
        for topic_name, msg_type_str, msg_class in TOPICS_FROM_ROBOT:
            local_pub = self.create_publisher(msg_class, topic_name, 10)
            humble_sub = humble_ros.Topic(self.ws_client, topic_name, msg_type_str)
            humble_sub.subscribe(self.make_inbound_callback(local_pub, msg_class))
            self.get_logger().info(f"Bridged Inbound: Robot WebSocket {topic_name} -> Local DDS")

    def make_outbound_callback(self, topic_name):
        def outbound_callback(msg):
            from rosidl_runtime_py import message_to_dict
            # Converts the custom ROS2 structure dynamically into a pure python dict
            msg_dict = message_to_dict(msg)
            humble_msg = humble_ros.Message(msg_dict)
            self.active_humble_publishers[topic_name].publish(humble_msg)
        return outbound_callback

    def make_inbound_callback(self, local_pub, msg_class):
        def inbound_callback(humble_msg_dict):
            from rosidl_runtime_py import set_message_fields
            ros2_msg = msg_class()
            try:
                # Dynamically maps the incoming WebSocket dict back into the custom ROS2 message object
                set_message_fields(ros2_msg, humble_msg_dict)
                local_pub.publish(ros2_msg)
            except Exception as e:
                self.get_logger().error(f"Failed parsing custom message field: {e}")
        return inbound_callback

    def shutdown(self):
        for humble_pub in self.active_humble_publishers.values():
            humble_pub.unadvertise()
        self.ws_client.terminate()

def main(args=None):
    rclpy.init(args=args)
    adapter = BidirectionalFoxgloveAdapter()
    try:
        rclpy.spin(adapter)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.shutdown()
        adapter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
