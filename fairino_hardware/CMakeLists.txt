cmake_minimum_required(VERSION 3.20)
project(fairino_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++23
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

# Set compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Set the libfairino version to use
set(LIBFAIRINO_FILE "libfairino.so.2.3.7")
set(LIBFAIRINO_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/${LIBFAIRINO_FILE}")

# Declare the library and its source files
add_library(fairino_hardware SHARED
  ./src/fairino_hardware_interface.cpp
)

# Add header files
target_include_directories(fairino_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
  $<INSTALL_INTERFACE:include/fairino_hardware>
)

# Link to other libraries
target_link_libraries(fairino_hardware
  "-Wl,-rpath,$ORIGIN"
  ${LIBFAIRINO_SOURCE_PATH}
  angles::angles
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)

# Install library
install(TARGETS fairino_hardware
  EXPORT fairino_hardware
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install included dependent library
install(FILES ${LIBFAIRINO_SOURCE_PATH}
  DESTINATION lib/${PROJECT_NAME}
)

# Set up version symbolic links. This must be an `install` command for compatibility with
# `colcon --symlink-install`. A plain `file` command will execute during configuration phase
# instead of during installation, and be missed by colcon.
install(CODE "
  file(CREATE_LINK
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${LIBFAIRINO_FILE}\"
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so.2\"
    SYMBOLIC
  )

  file(CREATE_LINK
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so.2\"
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libfairino.so\"
    SYMBOLIC
  )
")

ament_export_targets(fairino_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
