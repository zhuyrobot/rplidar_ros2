cmake_minimum_required(VERSION 3.15)
project(rplidar_ros2)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})
find_package(rosidl_default_generators REQUIRED) ####################add1
#2.Pro
set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)


include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)


add_executable(rplidarNode CMakeLists.txt  src/node.cpp ${RPLIDAR_SDK_SRC})

add_executable(rplidarNodeClient CMakeLists.txt  src/client.cpp)

install(TARGETS rplidarNode DESTINATION lib/${PROJECT_NAME}) ####################add2
install(TARGETS rplidarNodeClient DESTINATION lib/${PROJECT_NAME}) ####################add2
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) ####################add3
  
ament_package()###m####################add4
#colcon build --merge-install --base-path .. --cmake-args -DCOMMON_CMAKE=/root/app/include/cscv/cmaker/cscv.cmake
