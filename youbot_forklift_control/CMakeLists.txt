cmake_minimum_required(VERSION 2.8.3)
project(youbot_forklift_control)

find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES youbot_forklift_control
)


include_directories(include)

## Declare a cpp library
 add_library(youbot_forklift_control
    src/YoubotForkliftControl.cpp
    src/CommunicationNanotecMotorController.cpp
    src/DefaultLogger.cpp
    src/Logger.cpp
 )
 
target_link_libraries(youbot_forklift_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
 
install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

