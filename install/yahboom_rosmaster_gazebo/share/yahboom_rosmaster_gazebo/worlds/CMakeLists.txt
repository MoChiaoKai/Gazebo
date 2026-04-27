cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(amr_gui_plugin)

find_package(gazebo REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# 開啟 AUTOMOC，CMake 會自動尋找含有 Q_OBJECT 的標頭檔並編譯
set(CMAKE_AUTOMOC ON)

# 把 .cc 與 .hh 一起加進來編譯
add_library(amr_gui_plugin SHARED amr_gui_plugin.cc amr_gui_plugin.hh)
target_link_libraries(amr_gui_plugin ${GAZEBO_LIBRARIES} Qt5::Core Qt5::Widgets)