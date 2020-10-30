cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  	roscpp
    std_msgs
    sensor_msgs
    tf
   	nav_msgs
	message_generation
	dynamic_reconfigure
)


add_service_files(
	FILES
	computeDist.srv
)

add_message_files(
	FILES
	stat.msg
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
  
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lla2enu src/lla2enu.cpp)
target_link_libraries(lla2enu ${catkin_LIBRARIES})
add_dependencies(lla2enu ${catkin_EXPORTED_TARGETS})


add_executable(computeDist src/computeDist.cpp)
add_dependencies(computeDist ${catkin_EXPORTED_TARGETS})
target_link_libraries(computeDist ${catkin_LIBRARIES})

add_executable(publisher src/publisher.cpp)
add_dependencies(publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(publisher ${catkin_LIBRARIES})




