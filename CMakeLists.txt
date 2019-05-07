cmake_minimum_required(VERSION 2.8.3)
project(remote_car)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

# 自己装的opencv3的路径
set(OpenCV_DIR "/usr/local/opencv/share/OpenCV")
find_package(OpenCV 3.0 QUIET)
if (OpenCV_FOUND)
 message("Found Opencv : ${OpenCV_LIBRARIES}")
endif ()
if(NOT OpenCV_FOUND)
 find_package(OpenCV 2.4.3 QUIET)
 if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
 endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   car_data.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES remote_car
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
        serial
  ${catkin_INCLUDE_DIRS}
)

set(serial_SRCS
        serial/serial.cc
        serial/serial.h
        serial/v8stdint.h
        serial/unix.cc
        serial/unix.h
        serial/list_ports_linux.cc
        serial/open_serial.cpp serial/open_serial.h)

add_executable(send_message src/send_message.cpp ${serial_SRCS})

target_link_libraries(send_message
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
