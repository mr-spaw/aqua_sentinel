  cmake_minimum_required(VERSION 3.8)
  project(pipe_sensor_bridge)

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  # Compiler options
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # Find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  # Find mosquitto library (MQTT client)
  find_library(MOSQUITTO_LIB mosquitto REQUIRED)
  if(NOT MOSQUITTO_LIB)
    message(FATAL_ERROR "libmosquitto not found. Install with: sudo apt-get install libmosquitto-dev")
  endif()

  # nlohmann JSON library (header-only, usually in /usr/include)
  # If not found, install with: sudo apt-get install nlohmann-json3-dev
  find_path(JSON_INCLUDE_DIR nlohmann/json.hpp
    PATHS /usr/include /usr/local/include
    NO_DEFAULT_PATH
  )
  if(NOT JSON_INCLUDE_DIR)
    message(WARNING "nlohmann/json.hpp not found in standard locations")
    message(WARNING "Install with: sudo apt-get install nlohmann-json3-dev")
  endif()

  # Include directories
  include_directories(
    ${JSON_INCLUDE_DIR}
  )

  # Init Config Publisher 
  add_executable(init_config_publisher 
    src/init_config_publisher.cpp
  )

  # Link dependencies for init_config_publisher
  ament_target_dependencies(init_config_publisher 
    rclcpp 
    std_msgs
  )

  # Link mosquitto library
  target_link_libraries(init_config_publisher 
    ${MOSQUITTO_LIB}
  )

  # MQTT Subscriber Bridge 
  add_executable(mqtt_subscriber_bridge 
    src/mqtt_subscriber.cpp
  )

  # Link dependencies for mqtt_subscriber_bridge
  ament_target_dependencies(mqtt_subscriber_bridge 
    rclcpp 
    std_msgs
  )

  # Link mosquitto library
  target_link_libraries(mqtt_subscriber_bridge 
    ${MOSQUITTO_LIB}
  )

  # Installation 
  # Install executables
  install(TARGETS
    init_config_publisher
    mqtt_subscriber_bridge
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install launch files
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py"
  )

  # Install config files
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
  )

  # Install scripts with executable permissions
  install(PROGRAMS
    scripts/test_valve_control.sh
    scripts/monitor_sensors.sh
    scripts/maintenance.sh
    DESTINATION lib/${PROJECT_NAME}
  )

  # Also install to share for easy access
  install(PROGRAMS
    scripts/test_valve_control.sh
    scripts/monitor_sensors.sh
    scripts/maintenance.sh
    DESTINATION share/${PROJECT_NAME}/scripts
  )


  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()