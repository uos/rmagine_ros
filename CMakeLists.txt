cmake_minimum_required(VERSION 3.2)
project(imagine_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    dynamic_reconfigure
    mesh_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LidarModel.cfg
  cfg/CameraModel.cfg
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

find_package(imagine REQUIRED)
include_directories(${imagine_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS 
        # include
    CATKIN_DEPENDS 
        roscpp
        geometry_msgs
        sensor_msgs
        tf2_ros
    DEPENDS 
        imagine
)


include_directories(
    include
)

add_executable(lidar_simulator src/lidar_simulator.cpp)

add_dependencies(lidar_simulator 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_simulator
    ${catkin_LIBRARIES}
    ${imagine_LIBRARIES}
)


add_executable(camera_simulator src/camera_simulator.cpp)

add_dependencies(camera_simulator 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(camera_simulator
    ${catkin_LIBRARIES}
    ${imagine_LIBRARIES}
)



add_executable(mesh_publisher src/mesh_publisher.cpp)

add_dependencies(mesh_publisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(mesh_publisher
    ${catkin_LIBRARIES}
    ${imagine_LIBRARIES}
)
