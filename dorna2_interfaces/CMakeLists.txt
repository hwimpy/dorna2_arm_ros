cmake_minimum_required(VERSION 3.8)
project(dorna2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianPose.msg"
  "msg/IOState.msg"
  "msg/RobotStatus.msg"
  "srv/Connect.srv"
  "srv/Disconnect.srv"
  "srv/Halt.srv"
  "srv/JointMove.srv"
  "srv/LinearMove.srv"
  "srv/CircleMove.srv"
  "srv/Home.srv"
  "srv/SetJoint.srv"
  "srv/SetOutput.srv"
  "srv/SetPWM.srv"
  "srv/GetADC.srv"
  "srv/SetMotor.srv"
  "srv/SetAlarm.srv"
  "srv/SetPID.srv"
  "srv/GetPID.srv"
  "srv/SetTool.srv"
  "srv/SetToollength.srv"
  "srv/Probe.srv"
  "srv/SetEmergency.srv"
  "srv/GetRobotInfo.srv"
  "srv/Jog.srv"
  "srv/PlayScript.srv"
  "srv/SetGravity.srv"
  "srv/SetAxisRatio.srv"
  "srv/SetError.srv"
  DEPENDENCIES std_msgs
)

ament_package()
