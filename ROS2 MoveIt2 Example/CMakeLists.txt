cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

#include_directories(CML/inc CML/inc/can CML/inc/ecat)
#set(SOURCES CML/c/Amp.cpp CML/c/AmpFile.cpp)

add_library(CMLLib src/Amp.cpp src/AmpFile.cpp src/AmpFW.cpp src/AmpParam.cpp src/AmpPDO.cpp src/AmpPVT.cpp src/AmpStruct.cpp src/AmpUnits.cpp src/AmpVersion.cpp src/Can.cpp src/CanOpen.cpp src/CML.cpp src/CopleyIO.cpp src/CopleyIOFile.cpp src/CopleyNode.cpp src/ecatdc.cpp src/Error.cpp src/EtherCAT.cpp src/EventMap.cpp src/File.cpp src/Filter.cpp src/Firmware.cpp src/Geometry.cpp src/InputShaper.cpp src/IOmodule.cpp src/Linkage.cpp src/LSS.cpp src/Network.cpp src/Node.cpp src/Path.cpp src/PDO.cpp src/PvtConstAccelTrj.cpp src/PvtTrj.cpp src/Reference.cpp src/SDO.cpp src/Threads.cpp src/TrjScurve.cpp src/Utils.cpp src/Threads_posix.cpp src/ecat_linux.cpp src/can_copley.cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_link_libraries(hello_moveit CMLLib)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
