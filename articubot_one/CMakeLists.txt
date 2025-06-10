cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install launch files
install(DIRECTORY
  config
  description
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install all Python scripts
install(PROGRAMS
  scripts/detect_fire.py
  scripts/follow_fire.py
  scripts/follow_fire_fast.py
  scripts/debug_fire.py
  scripts/follow_fire_exploration.py
  scripts/detect_red.py
  scripts/test_move.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()