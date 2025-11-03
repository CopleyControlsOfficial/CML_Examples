cmake_minimum_required(VERSION 3.8)
project(copley_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)   # needed if using rclcpp library
find_package(std_msgs REQUIRED) # needed if sending messages over ROS2 framework
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(copley_node src/copley_node.cpp src/Amp.cpp src/AmpFile.cpp src/AmpFW.cpp src/AmpParam.cpp src/AmpPDO.cpp src/AmpPVT.cpp src/AmpStruct.cpp src/AmpUnits.cpp src/AmpVersion.cpp src/Can.cpp src/CanOpen.cpp src/CML.cpp src/CopleyIO.cpp src/CopleyIOFile.cpp src/CopleyNode.cpp src/Error.cpp src/EtherCAT.cpp src/ecatdc.cpp src/EventMap.cpp src/File.cpp src/Filter.cpp src/Firmware.cpp src/Geometry.cpp src/IOmodule.cpp src/Linkage.cpp src/Network.cpp src/Node.cpp src/Path.cpp src/PDO.cpp src/PvtConstAccelTrj.cpp src/PvtTrj.cpp src/Reference.cpp src/SDO.cpp src/Threads.cpp src/TrjScurve.cpp src/can_copley.cpp src/ecat_linux.cpp src/Threads_posix.cpp src/Utils.cpp src/InputShaper.cpp src/LSS.cpp)

ament_target_dependencies(copley_node rclcpp std_msgs)

target_include_directories(copley_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(copley_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS copley_node
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
