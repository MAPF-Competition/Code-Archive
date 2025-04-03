project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

#set(EXECUTABLE_OUTPUT_PATH ../)

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)

if (PYTHON)
    set(PYTHON_FLAG 1)
else ()
    set(PYTHON_FLAG 0)
endif ()


# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")

#SET(CMAKE_BUILD_TYPE "RELEASE")

IF (NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF ()

# more warnings
#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

file(GLOB SOURCES
        src/ActionModel.cpp
        src/common.cpp
        src/CompetitionSystem.cpp
        src/Entry.cpp
        src/Evaluation.cpp
        src/Grid.cpp
        src/Logger.cpp
        src/MAPFPlanner.cpp
        src/Simulator.cpp
        src/States.cpp
        src/TaskManager.cpp
        src/TaskScheduler.cpp

        inc/.*cpp

        default_planner/*.cpp

        # team solution
        Solution/settings.cpp
        Solution/Objects/*.cpp
        Solution/Objects/Basic/*.cpp
        Solution/Objects/Containers/*.cpp
        Solution/Objects/Environment/*.cpp

        Solution/Tools/tools.cpp

        Solution/Planner/*.cpp
        Solution/Planner/Solver/*.cpp
        Solution/Planner/Machine/*.cpp
        Solution/Planner/PIBT/*.cpp
        Solution/Planner/Default/*.cpp
        Solution/Planner/LaCAM/*.cpp

        Solution/Scheduler/*.cpp

        #Solution2/src/*.cpp
        #Solution2/src/LaCAM2/*.cpp
        #Solution2/src/LaCAM2/SUO2/*.cpp
        #Solution2/src/LNS/*.cpp
        #Solution2/src/LNS/Parallel/*.cpp
        #Solution2/src/RHCR/interface/*.cpp
        #Solution2/src/RHCR/main/*.cpp
        #Solution2/src/util/*.cpp
        )

include_directories("inc")

# default planner
include_directories("default_planner")

include_directories("Solution")

include_directories("Solution2")
include_directories("Solution2/inc")
include_directories("Solution2/src")

#ADD_DEFINITIONS(-DNDEBUG)

# Find Boost
#ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
SET(Boost_USE_STATIC_LIBS ON)
# iostreams
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)

include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

IF (PYTHON)
    message(STATUS "Python support is enabled")

    find_package(pybind11 REQUIRED)
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")

    # check if python/tmp exists, raise an error if it does not
    if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/python/tmp")
        message(FATAL_ERROR "Track is not configured for python bindings. Please run python/set_track.bash <combined|planner|scheduler> to configure the track before compiling.")
    endif ()

    include_directories("python/tmp")

    pybind11_add_module(MAPF "./src/Grid.cpp" "./src/States.cpp" "./python/tmp/MAPFbinding.cpp")
    target_link_libraries(MAPF PRIVATE)

    file(GLOB PY_SOURCES "python/tmp/*.cpp")
    list(FILTER PY_SOURCES EXCLUDE REGEX ".*python/MAPFbinding/.cpp")

    add_executable(lifelong ${PY_SOURCES} ${SOURCES})
    target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

    if (COPY_PY_PATH_CONFIG)
        file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
    endif ()


ELSE ()
    message(STATUS "Python support is disabled")

    add_executable(pibt_main Solution/Planner/PIBT/main/pibt_main.cpp ${SOURCES})

    add_executable(tester Solution/tester.cpp ${SOURCES})

    add_executable(lifelong src/driver.cpp ${SOURCES})

    add_executable(GGbuilder
            Solution/Objects/GraphGuidanceBuilder/gg_builder.cpp
            Solution/Objects/GraphGuidanceBuilder/graph_guidance_solver.cpp
            ${SOURCES})

    target_link_libraries(lifelong ${Boost_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})
    target_link_libraries(lifelong OpenMP::OpenMP_CXX)
    target_link_libraries(lifelong spdlog::spdlog)

    target_link_libraries(pibt_main ${Boost_LIBRARIES})
    target_link_libraries(tester ${Boost_LIBRARIES})
    target_link_libraries(GGbuilder ${Boost_LIBRARIES})

    target_link_libraries(pibt_main OpenMP::OpenMP_CXX)
    target_link_libraries(pibt_main spdlog::spdlog)

    target_link_libraries(tester OpenMP::OpenMP_CXX)
    target_link_libraries(tester spdlog::spdlog)

    target_link_libraries(GGbuilder OpenMP::OpenMP_CXX)
    target_link_libraries(GGbuilder spdlog::spdlog)

ENDIF ()


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


