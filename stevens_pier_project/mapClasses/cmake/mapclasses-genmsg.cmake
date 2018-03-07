# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mapclasses: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imapclasses:/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mapclasses_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg" NAME_WE)
add_custom_target(_mapclasses_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapclasses" "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mapclasses
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapclasses
)

### Generating Services

### Generating Module File
_generate_module_cpp(mapclasses
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapclasses
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mapclasses_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mapclasses_generate_messages mapclasses_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg" NAME_WE)
add_dependencies(mapclasses_generate_messages_cpp _mapclasses_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapclasses_gencpp)
add_dependencies(mapclasses_gencpp mapclasses_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapclasses_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mapclasses
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapclasses
)

### Generating Services

### Generating Module File
_generate_module_lisp(mapclasses
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapclasses
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mapclasses_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mapclasses_generate_messages mapclasses_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg" NAME_WE)
add_dependencies(mapclasses_generate_messages_lisp _mapclasses_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapclasses_genlisp)
add_dependencies(mapclasses_genlisp mapclasses_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapclasses_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mapclasses
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapclasses
)

### Generating Services

### Generating Module File
_generate_module_py(mapclasses
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapclasses
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mapclasses_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mapclasses_generate_messages mapclasses_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/mapClasses/msg/buff.msg" NAME_WE)
add_dependencies(mapclasses_generate_messages_py _mapclasses_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapclasses_genpy)
add_dependencies(mapclasses_genpy mapclasses_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapclasses_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapclasses)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapclasses
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(mapclasses_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(mapclasses_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapclasses)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapclasses
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(mapclasses_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(mapclasses_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapclasses)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapclasses\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapclasses
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(mapclasses_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(mapclasses_generate_messages_py sensor_msgs_generate_messages_py)
