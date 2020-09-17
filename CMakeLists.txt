
cmake_minimum_required(VERSION 2.8.0)
project(mydrone_gui_keyboard)


find_package(catkin REQUIRED COMPONENTS qt_build message_generation std_msgs roscpp)

generate_messages(DEPENDENCIES std_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

catkin_package(
LIBRARIES ros_tutorials_service
CATKIN_DEPENDS std_msgs roscpp mavros_msgs
)

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mydrone_gui_keyboard/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_compile_options(-std=c++11)

add_executable(mydrone_gui_keyboard ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(mydrone_gui_keyboard ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS mydrone_gui_keyboard RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
