cmake_minimum_required(VERSION 3.0.2)
project(cyphal_bridge)

add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

add_compile_definitions(NUNAVUT_SUPPORT_LANGUAGE_OPTION_CAST_FORMAT=2368206204)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED)
find_library(BOOST_STACKTRACE_BACKTRACE_LIBRARY NAMES boost_stacktrace_backtrace)

if(DEFINED ENV{<CYPHAL_DIR>})
    set(CYPHAL_DIR $ENV{CYPHAL_DIR})
else()
    get_filename_component(CYPHAL_DIR
                           "libs/libcyphal"
                           ABSOLUTE)
endif()
message("${CMAKE_CURRENT_LIST_DIR} | Using <${CYPHAL_DIR}> as directory of libcyphal")

add_subdirectory(${CYPHAL_DIR} ${PROJECT_BINARY_DIR}/build/libcyphal)

#add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
#add_compile_options(-ffunction-sections -fdata-sections)
#add_compile_options(-Ofast)

file(GLOB_RECURSE SOURCES "src/*.*" "libs/utils.cpp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

## Generate messages in the 'msg' folder
add_message_files(FILES
   HMILed.msg
   HMIBeeper.msg
)
add_service_files(FILES
    ResetDrive.srv
    PowerReset.srv
)
 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  include
  libs
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${CYPHAL_DIR}
  ${COMMON_LIBS_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Eigen3::Eigen libcyphal dl)

install(TARGETS libcyphal
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
