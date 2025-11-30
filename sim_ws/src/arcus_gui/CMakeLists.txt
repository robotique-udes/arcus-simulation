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
add_executable(arcus_gui
    src/gui.cpp
    src/MainWindow.cpp
    src/QWidget/QExampleWidget/QExampleWidget.cpp
)

# ROS2 dependencies
ament_target_dependencies(arcus_gui rclcpp nav_msgs)

# Qt6 linking (plain style, no PRIVATE keyword)
target_link_libraries(arcus_gui Qt6::Core Qt6::Widgets)

# Qt target properties
set_target_properties(arcus_gui PROPERTIES AUTOMOC ON AUTORCC ON)

install(TARGETS arcus_gui
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
