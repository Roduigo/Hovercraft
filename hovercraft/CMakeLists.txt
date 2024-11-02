cmake_minimum_required(VERSION 3.0.2)
project(hovercraft)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge 
  sensor_msgs
  message_generation #pacote que cria novos tipos de mensagens
)

add_message_files(
  FILES
  #add files .msg here
)

#dependencias necessárias para gerar a mensagem personalizada
generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


