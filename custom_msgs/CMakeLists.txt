cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  "msg/Canvas.msg"
  "msg/Names.msg"
  "msg/Agents.msg"
  "msg/Files.msg"
  "msg/Fbdl.msg"
  "msg/Inputs.msg"
  "msg/Outputs.msg"
  "msg/Obstacles.msg"
  "msg/SpecObject.msg"
  "msg/Target.msg"
  "msg/Distance.msg"
  "msg/Rays.msg"
  "srv/NameService.srv"
  "srv/Delete.srv"
  "srv/NewAgent.srv"
  "srv/DeleteObstacle.srv"
  "srv/CanvasDimensions.srv"
  "srv/CreateObstacle.srv"
  "srv/CreateSpecialObject.srv"
  "srv/ChangeDimensions.srv"
  "srv/ChangeColor.srv"
  "srv/Load.srv"
  "srv/Agents.srv"
  "srv/Follow.srv"
  "srv/SendState.srv"
  "srv/Error.srv"
  "srv/Stop.srv"
  "srv/Universes.srv"
  "srv/Fbdl.srv"
  "srv/Weight.srv"
 )
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
