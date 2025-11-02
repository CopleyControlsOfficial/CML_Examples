cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# talker executable doesn't use the Copley Motion Library (CML).
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# listener executable uses the Copley Motion Library (CML).
add_executable(listener src/subscriber_member_function.cpp src/Amp.cpp src/AmpFile.cpp src/AmpFW.cpp src/AmpParam.cpp src/AmpPDO.cpp src/AmpPVT.cpp src/AmpStruct.cpp src/AmpUnits.cpp src/AmpVersion.cpp src/Can.cpp src/CanOpen.cpp src/CML.cpp src/CopleyIO.cpp src/CopleyIOFile.cpp src/CopleyNode.cpp src/Error.cpp src/EtherCAT.cpp src/ecatdc.cpp src/EventMap.cpp src/File.cpp src/Filter.cpp src/Firmware.cpp src/Geometry.cpp src/IOmodule.cpp src/Linkage.cpp src/Network.cpp src/Node.cpp src/Path.cpp src/PDO.cpp src/PvtConstAccelTrj.cpp src/PvtTrj.cpp src/Reference.cpp src/SDO.cpp src/Threads.cpp src/TrjScurve.cpp src/can_copley.cpp src/ecat_linux.cpp src/Threads_posix.cpp src/Utils.cpp src/InputShaper.cpp src/LSS.cpp)

ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
