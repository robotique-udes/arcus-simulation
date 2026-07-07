# How to run rosbag in simu

- Record your rosbag
- Run the cleaner script to fix the time stamp and remove map to odom

```bash
    python3 fix_bag_for_simu.py input_bag/ output_bag/
```
- Run the simu with the custom launch file ros_bag_sim_launch.py
- Run particle filter with sim:=true argument
- Run the ros bag with this command
```bash
    ros2 bag play bag_name/ --clock
```