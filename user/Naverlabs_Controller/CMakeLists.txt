# cmake_minimum_required(VERSION 3.1)

# project(lcm_cpp_example)

# find_package(lcm REQUIRED)
# include(${LCM_USE_FILE})
# include_directories(${PROJECT_SOURCE_DIR})
# include_directories("../../robot/include")
# include_directories("../../common/include/")
# include_directories("../../lcm-types/cpp")
# include_directories("../../")

# Put all message definition files in the type directory in one list
# FILE(GLOB example_message_definitions "${CMAKE_CURRENT_LIST_DIR}/../types/*.lcm")

# Generate headers from message definition
# lcm_wrap_types(CPP_HEADERS cpp_headers
#   ${example_message_definitions})

# Create library from all the messages
# lcm_add_library(example_messages-cpp CPP ${cpp_headers})
# target_include_directories(example_messages-cpp INTERFACE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

# Create executables for the three example programs, linking all of them to our
# messages library and lcm

# add_executable(demo_lowlevel_listener "demo_lowlevel_listener.cpp")
# target_link_libraries(demo_lowlevel_listener example_messages-cpp ${LCM_NAMESPACE}lcm)


# add_executable(demo_state_publisher demo_state_publisher.cpp Lowlevel_Controller.cpp)
# target_link_libraries(demo_state_publisher robot biomimetics dynacore_param_handler qpOASES)

include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_executable(naverlabs_ctrl main.cpp Lowlevel_Controller.cpp)
target_link_libraries(naverlabs_ctrl robot biomimetics dynacore_param_handler qpOASES)
