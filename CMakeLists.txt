cmake_minimum_required(VERSION 2.8.3)
project(sdl2_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
)

roslint_cpp()

catkin_package()

find_package(SDL2 REQUIRED)
# https://stackoverflow.com/questions/45730098/cmake-leading-or-trailing-whitespace-policy-cmp0004
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
# No FindSDL2_mixer.cmake in Ubuntu 16.04?
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
  # SDL2_mixer
  ${catkin_LIBRARIES}
)
