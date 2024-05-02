cmake_minimum_required(VERSION 3.5)
project(gbeam2_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######## ------------ Find Dependencies ------------ ########
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gbeam2_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)





###### -------- Define a library target called gbeam2_library --------#######

# add_library(polytope_fcn src/polytope_fcn.cpp)

# add_library(graph_fcn src/graph_fcn.cpp)

# add_library(exploration_fcn src/exploration_fcn.cpp)

add_library(library_fcn src/library_fcn.cpp)


####### ------- Adding targets ------- #######

# target_include_directories(
#   polytope_fcn
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# target_include_directories(
#   graph_fcn
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_include_directories(
  library_fcn
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

####### -------- Linking to dependencies ------ #######

set(my_dep
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "visualization_msgs"
  "gbeam2_interfaces"
  "tf2_ros"
  "tf2_geometry_msgs")


# ament_target_dependencies(polytope_fcn PUBLIC ${my_dep})

# ament_target_dependencies(graph_fcn PUBLIC ${my_dep})

# ament_target_dependencies(exploration_fcn PUBLIC ${my_dep})

ament_target_dependencies(library_fcn PUBLIC ${my_dep})


# Let the compiler search for the headers in the include directory
include_directories(include)




#target_link_libraries(polytope_fcn ${rclcpp_LIBRARIES})
################################################ target_link_libraries(polytope_fcn ${rclcpp LIBRARIES})
# this line to export the library
# ament_export_targets(polytope_fcn HAS_LIBRARY_TARGET)
# ament_export_targets(exploration_fcn HAS_LIBRARY_TARGET)
# ament_export_targets(graph_fcn HAS_LIBRARY_TARGET)

####### ---------- Install include directory ------------######
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library
install(
  TARGETS  library_fcn                             #polytope_fcn graph_fcn exploration_fcn
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_dependencies(rosidl_default_runtime)

ament_package()