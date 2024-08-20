cmake_minimum_required(VERSION 3.0.2)
project(omni_vision)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omni_vision
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS YAML_CPP
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(omni_rect_node src/omni_rect.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(omni_rect_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
)

# Mark executable files for installation
install(TARGETS omni_rect_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
