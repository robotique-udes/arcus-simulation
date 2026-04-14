cmake_minimum_required(VERSION 3.8)
project(arcus_gui)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
find_package(arcus_msgs REQUIRED)

# Qt auto-processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  src
  ui/include
)

# Optional: custom UI generation (keep if needed)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/ui_generated
    COMMAND bash ${CMAKE_SOURCE_DIR}/ui/generate_ui.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating UI files..."
)
add_custom_target(run_generate_ui ALL DEPENDS ${CMAKE_BINARY_DIR}/ui_generated)

# Executable
add_executable(main_gui src/gui.cpp
    src/MainWindow.cpp
    src/QWidget/QExampleWidget/QExampleWidget.cpp
    src/QWidget/QArcusMaster/QArcusMaster.cpp
    src/QWidget/QLocalNodes/QLocalNodes.cpp
    src/QWidget/QMapRacelineHelpers/QMapRacelineHelpers.cpp
    src/QWidget/QPurePursuitWidget/QPurePursuit.cpp
    src/QWidget/QArcusMaster/QSafetyWidget/QSafetyWidget.cpp
    src/Global/Helper/QTopicSelector/QTopicSelector.cpp
    src/Global/Helper/QProcessHandler/QProcessHandler.cpp
)

# ROS2 dependencies
ament_target_dependencies(main_gui rclcpp nav_msgs arcus_msgs)

# Qt6 linking (plain style, no PRIVATE keyword)
target_link_libraries(main_gui Qt6::Core Qt6::Widgets)

# Qt target properties
set_target_properties(main_gui PROPERTIES AUTOMOC ON AUTORCC ON)

install(TARGETS main_gui
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
