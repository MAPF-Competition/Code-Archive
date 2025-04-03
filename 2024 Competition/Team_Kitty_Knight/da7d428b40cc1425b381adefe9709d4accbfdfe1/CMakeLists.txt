project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

# whether to use shared memory heuristics
# add_definitions(-DSHARED_MEMORY_HEURISTICS)

# if set, use dev mode, which has a lot of logging and timing.
# set(DEV ON)
# if set, use action model with no rotation.
# set(NO_ROT ON)

# set(TrafficFlow ON)

IF(DEV)
    message(STATUS "Dev mode is enabled")
    add_definitions(-DDEV)
ENDIF()


IF(TrafficFlow)
    message(STATUS "using TrafficFlow")
    add_definitions(-DTrafficFlow)
ENDIF()

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)

if(PYTHON)
    set(PYTHON_FLAG 1)
else()
    set(PYTHON_FLAG 0)
endif()


# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")

IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")

if (TrafficFlow)
    include_directories("default_planner")
    file(GLOB_RECURSE SOURCES "src/**.cpp" "default_planner/**.cpp")
else()
    # default planner
    #include_directories("my_planner/inc")
    include_directories("wppl_planner/inc")

    #file(GLOB_RECURSE SOURCES "src/**.cpp" "my_planner/**.cpp")
    file(GLOB_RECURSE SOURCES "src/**.cpp" "wppl_planner/**.cpp")
endif()

list(FILTER SOURCES EXCLUDE REGEX "(src/py_driver.cpp)|(src/PyCompetitionSystem.cpp)")

# Print the list
message("List of files:")
foreach(FILE ${SOURCES})
    message("  ${FILE}")
endforeach()


# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
#SET(Boost_USE_STATIC_LIBS ON)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup iostreams)

#include_directories( ${Boost_INCLUDE_DIRS} )
#include_directories("spdlog/include" )

find_package(pybind11 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)
# find_package(spdlog REQUIRED)

# -Wfatal-errors will stop the compilation after the first error
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors") 

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -fpermissive")

# IF(PYTHON)
#     message(STATUS "Python support is enabled")

#     find_package(pybind11 REQUIRED)
#     set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")

#      # check if python/tmp exists, raise an error if it does not
#     if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/python/tmp")
#         message(FATAL_ERROR "Track is not configured for python bindings. Please run python/set_track.bash <combined|planner|scheduler> to configure the track before compiling.")
#     endif()

#     include_directories("python/tmp")

#     pybind11_add_module(MAPF "./src/Grid.cpp"   "./src/States.cpp" "./python/tmp/MAPFbinding.cpp")
#     target_link_libraries(MAPF PRIVATE ) 

#     file(GLOB PY_SOURCES "python/tmp/*.cpp")
#     list(FILTER PY_SOURCES EXCLUDE REGEX ".*python/MAPFbinding\.cpp")

#     add_executable(lifelong  ${PY_SOURCES} ${SOURCES})
#     target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})
#     target_link_libraries(lifelong PRIVATE OpenMP::OpenMP_CXX)
#     target_link_libraries(lifelong PRIVATE spdlog::spdlog)
#     target_link_libraries(lifelong PRIVATE TBB::tbb)
#     target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

#     if (COPY_PY_PATH_CONFIG)
#         file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
#     endif()


# ELSE()
message(STATUS "Python support is disabled")

add_executable(lifelong ${SOURCES} "src/driver.cpp")
target_link_libraries(lifelong PRIVATE ${Boost_LIBRARIES})
target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})
target_link_libraries(lifelong PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(lifelong PRIVATE rt)
# target_link_libraries(lifelong spdlog::spdlog)
target_link_libraries(lifelong PRIVATE TBB::tbb)
target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})
# ENDIF()

# if (TrafficFlow)

# else()
# # py_driver
# pybind11_add_module(py_driver ${SOURCES} "src/py_driver.cpp" "src/PyCompetitionSystem.cpp")
# target_link_libraries(py_driver PRIVATE ${Boost_LIBRARIES})
# target_link_libraries(py_driver PRIVATE pybind11::embed ${Boost_LIBRARIES})
# target_link_libraries(py_driver PRIVATE OpenMP::OpenMP_CXX)
# target_link_libraries(py_driver PRIVATE rt)
# # target_link_libraries(lifelong spdlog::spdlog)
# target_link_libraries(py_driver PRIVATE TBB::tbb)
# target_compile_definitions(py_driver PRIVATE PYTHON=${PYTHON_FLAG})
# endif()



add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/lifelong
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
)


set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_BINARY_DIR}/lifelong;
     ${CMAKE_BINARY_DIR}/build;
     ${CMAKE_BINARY_DIR}/*.so"
)



