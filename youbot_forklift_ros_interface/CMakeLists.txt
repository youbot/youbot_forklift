cmake_minimum_required(VERSION 2.8.3)
project(youbot_forklift_ros_interface)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	sensor_msgs
	actionlib_msgs 
	actionlib
	youbot_forklift_control
)

add_action_files(DIRECTORY action FILES GoToPosition.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
	CATKIN_DEPENDS actionlib_msgs)

include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${youbot_forklift_control_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(youbot_forklift_ros_hello_world
	src/ForkliftRosHelloWorld.cpp)
target_link_libraries(youbot_forklift_ros_hello_world ${catkin_LIBRARIES})

add_executable(youbot_forklift_ros_interface 
	src/ForkliftRosInterface.cpp
	src/RosLogger.cpp)
target_link_libraries(youbot_forklift_ros_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(youbot_forklift_ros_hello_world ${youbot_forklift_ros_interface_EXPORTED_TARGETS})
add_dependencies(youbot_forklift_ros_interface ${youbot_forklift_ros_interface_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
