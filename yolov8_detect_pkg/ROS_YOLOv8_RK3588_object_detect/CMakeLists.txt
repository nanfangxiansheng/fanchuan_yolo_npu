cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED TARGET_SOC OR "${TARGET_SOC}" STREQUAL "")
  set(TARGET_SOC rk3588 CACHE STRING "RK SoC, e.g. rk3588")
endif()

project(ros_yolov8_rk3588_object_detect)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(YOLOV8_LABEL_TXT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/model/A_B_classgarbage.txt")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/label_path_config.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/label_path_config.h
  @ONLY)

if(DEFINED CATKIN_DEVEL_PREFIX OR DEFINED CATKIN_BUILD_BINARY_PACKAGE)
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge std_msgs roslib yolov8_ros_msgs)
  catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv_bridge std_msgs roslib yolov8_ros_msgs)
endif()

if(ENABLE_ASAN)
  message(STATUS "BUILD WITH ADDRESS SANITIZER")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
  set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif()

set(rknpu_yolov8_file src/yolov8.cc)
set(RKNNRT_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/rknn_lib)
link_directories(${RKNNRT_LIB_PATH})

if(TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
  add_definitions(-DRV1106_1103)
  set(rknpu_yolov8_file rknpu2/yolov8_rv1106_1103.cc)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/allocator/dma)
endif()

if(TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
  add_definitions(-DRKNPU1)
  set(rknpu_yolov8_file rknpu1/yolov8.cc)
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils/ utils.out)

set(CMAKE_INSTALL_PATH "$ORIGIN/../lib")

add_executable(rknn_yolov8_demo
  src/main.cc
  src/postprocess.cc
  ${rknpu_yolov8_file}
)

target_include_directories(rknn_yolov8_demo PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIBRKNNRT_INCLUDES})

target_link_libraries(rknn_yolov8_demo
  imageutils
  fileutils
  imagedrawing
  ${OpenCV_LIBS}
  ${LIBRKNNRT}
  rknnrt
  dl
)

if(CMAKE_SYSTEM_NAME STREQUAL "Android")
  target_link_libraries(rknn_yolov8_demo log)
endif()

message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  target_link_libraries(rknn_yolov8_demo Threads::Threads)
endif()

if(DEFINED CATKIN_DEVEL_PREFIX OR DEFINED CATKIN_BUILD_BINARY_PACKAGE)
  add_executable(yolov8_usb_cam_node
    src/yolov8_usb_cam_node.cc
    src/postprocess.cc
    ${rknpu_yolov8_file}
  )

  target_include_directories(yolov8_usb_cam_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${LIBRKNNRT_INCLUDES}
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(yolov8_usb_cam_node
    imageutils
    fileutils
    imagedrawing
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    rknnrt
    dl
    ${catkin_LIBRARIES}
  )

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(yolov8_usb_cam_node Threads::Threads)
  endif()

  install(TARGETS rknn_yolov8_demo yolov8_usb_cam_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  install(TARGETS rknn_yolov8_demo DESTINATION .)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/model/bus.jpg DESTINATION model)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/model/4classgarbage.txt DESTINATION model)
  file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/model/*.rknn")
  install(FILES ${RKNN_FILES} DESTINATION model)
endif()
