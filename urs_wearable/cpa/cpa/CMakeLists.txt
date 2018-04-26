cmake_minimum_required(VERSION 3.0)
project(cpa+)

# Either both of them are OFF, or only one of them should be set to ON
option(RELEASE "Set ON to build in Release mode" ON)
option(DEBUG "Set ON to build in Debug mode" OFF)

if(RELEASE)
  set(CMAKE_BUILD_TYPE Release)
elseif(DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF) # Set OFF to use -std=c++11, otherwise it uses -std=gnu++11 by default

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

BISON_TARGET(parser lcp.y ${CMAKE_CURRENT_SOURCE_DIR}/lcp.tab.cpp)
FLEX_TARGET(scanner lcp.lex ${CMAKE_CURRENT_SOURCE_DIR}/lex.yy.cpp)
ADD_FLEX_BISON_DEPENDENCY(scanner parser)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../rosbridgecpp)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../rosbridgecpp rosbridgecpp)

add_executable(${PROJECT_NAME}
  action.cpp
  main.cpp
  planner.cpp
  reader.cpp
  state.cpp
  
  ${BISON_parser_OUTPUTS}
  ${FLEX_scanner_OUTPUTS}
)

target_link_libraries(${PROJECT_NAME} ${FLEX_LIBRARIES} ${BISON_LIBRARIES} rosbridgecpp)
