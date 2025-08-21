cmake_minimum_required(VERSION 3.10)

# set the project name
project(icm20948_ros2)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-fPIC -g -pedantic -Wall -Wextra")

if(LOG) 
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLOG=1")
endif()

# Check if this is being built as ROS2 package
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    # ROS2 build
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    
    # add the I2C submodule - MUST be built first
    if (CUSTOM_I2C)
        include_directories(${CUSTOM_I2C}/src)
    else()
        add_subdirectory(I2C)
        include_directories(I2C/src)
    endif()
    
    include_directories(src src/ahrs)
    
    # Build the actual library with proper I2C linking
    add_library(ICM20948 SHARED src/ahrs/ahrs_math.cpp src/ahrs/madgwick_ahrs.cpp src/ahrs/simple_ahrs.cpp src/icm20948.cpp)
    target_link_libraries(ICM20948 I2C)
    
    # Create ROS2 executable with both libraries
    add_executable(icm20948_ros2_node src/icm20948_ros2_node.cpp)
    target_link_libraries(icm20948_ros2_node ICM20948 I2C)
    
    # ROS2 dependencies
    ament_target_dependencies(icm20948_ros2_node
        rclcpp
        sensor_msgs
    )
    
    # Install executable
    install(TARGETS icm20948_ros2_node
        DESTINATION lib/${PROJECT_NAME}
    )
    
    # Install libraries properly - this is crucial!
    install(TARGETS ICM20948 I2C
        EXPORT ICM20948Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
    
    # Install header files for other packages
    install(DIRECTORY src/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
    
    install(DIRECTORY I2C/src/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
    )
    
    # Install launch files
    install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
        FILES_MATCHING PATTERN "*.launch.py"
    )
    
    # Install config files
    install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
        FILES_MATCHING PATTERN "*.yaml"
    )
    
    # Export the targets so other packages can find them
    install(EXPORT ICM20948Targets
        FILE ICM20948Targets.cmake
        NAMESPACE ICM20948::
        DESTINATION lib/cmake/${PROJECT_NAME}
    )
    
    # Export dependencies for other packages
    ament_export_targets(ICM20948Targets HAS_LIBRARY_TARGET)
    ament_export_libraries(ICM20948 I2C)
    ament_export_include_directories(include)
    ament_export_dependencies(rclcpp sensor_msgs)
    
    ament_package()
else()
    # Original build (non-ROS2) - keep original project name for backward compatibility
    project(ICM20948)
    
    # add the I2C submodule
    if (CUSTOM_I2C)
        include_directories(${CUSTOM_I2C}/src)
    else()
        add_subdirectory(I2C)
        include_directories(I2C/src)
    endif()

    include_directories(src)

    # Build the actual library
    add_library(ICM20948 SHARED src/ahrs/ahrs_math.cpp src/ahrs/madgwick_ahrs.cpp src/ahrs/simple_ahrs.cpp src/icm20948.cpp)
    target_link_libraries(ICM20948 I2C)

    # add the test application
    add_executable(test_icm20948 tests/icm20948_app.cpp)
    target_link_libraries(test_icm20948 ICM20948 I2C)
endif()
