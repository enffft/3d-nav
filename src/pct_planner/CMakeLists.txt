cmake_minimum_required(VERSION 3.0.2)
project(pct_planner)

# 查找catkin和所需的其他ROS组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

# 设置第三方项目路径
set(THIRDPARTY_ROOT ${PROJECT_SOURCE_DIR}/PCT_planner-main)
set(PLANNER_DIR ${THIRDPARTY_ROOT}/planner)

# 检查路径是否存在
if(NOT EXISTS ${PLANNER_DIR})
    message(FATAL_ERROR "Planner directory not found: ${PLANNER_DIR}")
endif()

# 自定义目标：构建第三方依赖 (GTSAM 和 OSQP)
add_custom_target(build_thirdparty
    WORKING_DIRECTORY ${PLANNER_DIR}
    COMMAND /bin/bash build_thirdparty.sh
    COMMENT "Building third-party dependencies (GTSAM and OSQP)..."
)

# 自定义目标：构建主项目
add_custom_target(build_main
    WORKING_DIRECTORY ${PLANNER_DIR}
    COMMAND /bin/bash build.sh
    COMMENT "Building main motion planning libraries..."
)

# 确保构建顺序：先第三方依赖，后主项目
add_dependencies(build_main build_thirdparty)

# 创建一个虚拟目标，用于表示整个wrapper的构建
add_custom_target(${PROJECT_NAME}_build ALL)
add_dependencies(${PROJECT_NAME}_build build_main)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pct_planner
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

# 安装生成的共享库（如果目录存在）
if(EXISTS "${PLANNER_DIR}/lib")
    install(DIRECTORY ${PLANNER_DIR}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "*.so"
    )
else()
    message(WARNING "Directory ${PLANNER_DIR}/lib does not exist. Skipping library installation.")
endif()

# 安装GTSAM库（如果目录存在）
if(EXISTS "${PLANNER_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib")
    install(DIRECTORY ${PLANNER_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "*.so"
    )
else()
    message(WARNING "Directory ${PLANNER_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib does not exist. Skipping GTSAM installation.")
endif()

# 安装OSQP库（如果目录存在）
if(EXISTS "${PLANNER_DIR}/lib/3rdparty/osqp/install/lib")
    install(DIRECTORY ${PLANNER_DIR}/lib/3rdparty/osqp/install/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "*.so"
    )
else()
    message(WARNING "Directory ${PLANNER_DIR}/lib/3rdparty/osqp/install/lib does not exist. Skipping OSQP installation.")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pct_planner.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pct_planner_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(PROGRAMS
  PCT_planner-main/tomography/scripts/tomography.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pct_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
