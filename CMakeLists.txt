cmake_minimum_required(VERSION 2.8)
project(test_image_view)

find_package(catkin REQUIRED COMPONENTS 
    camera_calibration_parsers 
    cv_bridge dynamic_reconfigure 
    image_transport 
    message_filters 
    message_generation 
    nodelet 
    rosconsole 
    roscpp 
    std_srvs 
    stereo_msgs
)

generate_dynamic_reconfigure_options(cfg/ImageView.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

# Extra tools

# Deal with the GUI's
if(ANDROID)
  return()
endif()

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_HB REQUIRED harfbuzz)
include_directories(${PC_HB_INCLUDE_DIRS})

# Nodelet library
add_library(image_view src/nodelets/image_nodelet.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES}
                                 ${GTK_LIBRARIES}
                                 ${GTK2_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${Boost_LIBRARIES}
)
add_dependencies(image_view ${PROJECT_NAME}_gencfg)
install(TARGETS image_view
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Image viewers
add_executable(image_view_exe src/nodes/image_view.cpp)
add_dependencies(image_view_exe ${PROJECT_NAME}_gencfg)
SET_TARGET_PROPERTIES(image_view_exe PROPERTIES OUTPUT_NAME image_view)
target_link_libraries(image_view_exe ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
                                     ${Boost_LIBRARIES}
)

install(TARGETS image_view_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
