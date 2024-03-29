cmake_minimum_required(VERSION 3.0.2)
project(sdl2_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
)

set(ROSLINT_PYTHON_OPTS "--ignore=W503" "1>&2")
roslint_python()

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references,-build/c++11"
)

roslint_cpp()

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(SDL2 REQUIRED)

# https://stackoverflow.com/questions/45730098/cmake-leading-or-trailing-whitespace-policy-cmp0004
# string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)

# No FindSDL2_mixer.cmake in Ubuntu 20.04
# find_package(SDL2_mixer REQUIRED)
# Use pkg-config instead
find_package(PkgConfig)
# Using an underscore like SDL2_mixer doesn't work
pkg_check_modules(SDL2mixer REQUIRED SDL2_mixer)

include_directories(
  ${SDL2_INCLUDE_DIRS}
  ${SDL2mixer_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sdl2_mixer src/sdl2_mixer.cpp)
target_link_libraries(sdl2_mixer
  ${SDL2_LIBRARIES}
  ${SDL2mixer_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(view_image src/view_image.cpp)
target_link_libraries(view_image
  ${SDL2_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Rust parts
find_package(Corrosion)
if(Corrosion_FOUND)
corrosion_import_crate(MANIFEST_PATH Cargo.toml)

function(rust_target target_name)
  add_custom_target(${target_name}_copy_to_devel ALL
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target_name}> ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/${target_name}
  )
  add_dependencies(${target_name}_copy_to_devel ${target_name})
  corrosion_install(
    TARGETS ${target_name}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction(rust_target)

rust_target(view_image_rs)
endif(Corrosion_FOUND)
