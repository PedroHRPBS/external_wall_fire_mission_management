cmake_minimum_required(VERSION 2.8.3)
project(external_wall_fire)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  positioning_system
  common_srv
  flight_scenario
)

# add_message_files(
#   FILES
#   PID_param.msg
#   Waypoint.msg
# )

add_service_files(
  FILES
  Flight_Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime positioning_system common_srv flight_scenario
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Navio2_cpp/*.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
