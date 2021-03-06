cmake_minimum_required(VERSION 2.8.3)
project(openni2_camera)

find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure image_transport  nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig)

pkg_check_modules(PC_OPENNI2 libopenni2)
if (NOT PC_OPENNI2_FOUND)
  # find_package(libOpenNI2 PATHS /usr/local/lib/ni /usr/local/include/ni)
  # pkg_check_modules(PC_OPENNI2 REQUIRED libOpenNI2)
  # NOTE: Install openni2 with: brew install brewsci/science/openni2
  set(libopenni2_INCLUDE_DIRS $ENV{OPENNI2_INCLUDE})
  set(libopenni2_LIBRARY_DIRS $ENV{OPENNI2_REDIST})
  find_library(libopenni2_LIBRARY NAMES OpenNI2 libOpenNI2 PATHS ${libopenni2_LIBRARY_DIRS})
  link_libraries(${libopenni2_LIBRARY})
endif()

generate_dynamic_reconfigure_options(cfg/OpenNI2.cfg)
add_service_files(FILES
  GetSerial.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES openni2_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  DEPENDS libopenni2
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${libopenni2_INCLUDE_DIRS}
                    )

link_directories(${libopenni2_LIBRARY_DIRS})

add_library(openni2_wrapper
   src/openni2_convert.cpp
   src/openni2_device.cpp
   src/openni2_device_info.cpp
   src/openni2_timer_filter.cpp
   src/openni2_frame_listener.cpp
   src/openni2_device_manager.cpp
   src/openni2_exception.cpp
   src/openni2_video_mode.cpp
)
target_link_libraries(openni2_wrapper ${catkin_LIBRARIES} ${libopenni2_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(test_wrapper test/test_wrapper.cpp )
target_link_libraries(test_wrapper openni2_wrapper ${Boost_LIBRARIES})

add_library(openni2_driver_lib
   src/openni2_driver.cpp
)
target_link_libraries(openni2_driver_lib openni2_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(openni2_driver_lib ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(openni2_camera_nodelet
   ros/openni2_camera_nodelet.cpp
)
target_link_libraries(openni2_camera_nodelet openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(openni2_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(openni2_camera_node
   ros/openni2_camera_node.cpp
)
target_link_libraries(openni2_camera_node openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(openni2_camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(list_devices
   src/list_devices.cpp
)
target_link_libraries(list_devices openni2_wrapper)

if (UNIX AND NOT APPLE)
  add_executable(usb_reset src/usb_reset.c)
  set(ADDITIONAL_EXECUTABLES "usb_reset")
endif()

install(TARGETS openni2_wrapper openni2_camera_nodelet openni2_camera_node list_devices openni2_driver_lib ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES openni2_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
