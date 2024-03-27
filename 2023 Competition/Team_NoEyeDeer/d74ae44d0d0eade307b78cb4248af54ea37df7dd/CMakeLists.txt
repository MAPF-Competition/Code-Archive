
cmake_minimum_required(VERSION 3.16)
project(lifelong)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
# set(CMAKE_SHARED_LIBRARY_C_FLAGS "${CMAKE_SHARED_LIBRARY_C_FLAGS} -pg")
# set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -pg")

# add_compile_options(-g -fno-omit-frame-pointer -O0)

set(CMAKE_CXX_STANDARD 17)

if(APPLE)
    set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
    set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")
    set(OpenMP_C_FLAGS "-fopenmp=libomp")
    set(OpenMP_CXX_FLAGS "-fopenmp=libomp")
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/libomp/")
endif()

find_package(OpenMP REQUIRED)

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)

if(PYTHON)
    set(PYTHON_FLAG 1)
else()
    set(PYTHON_FLAG 0)
endif()


option(PRINT_DEBUG "print debug info to terminal" OFF)

if(PRINT_DEBUG)
    ADD_DEFINITIONS(-D DEBUG)
endif()

# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")

IF(NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()



include_directories("inc")

# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
#SET(Boost_USE_STATIC_LIBS ON)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)

include_directories( ${Boost_INCLUDE_DIRS} )

IF(PYTHON)
    include_directories("python")
    message(STATUS "Python support is enabled")

    find_package(pybind11 REQUIRED)
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")


    pybind11_add_module(MAPF "./src/Grid.cpp"   "./src/States.cpp" "./python/MAPFbinding.cpp")
    target_link_libraries(MAPF PRIVATE ) 

    file(GLOB SOURCES "src/*.cpp")

    file(GLOB PY_SOURCES "python/*.cpp")
    list(FILTER PY_SOURCES EXCLUDE REGEX ".*python/MAPFbinding\.cpp")

    add_executable(lifelong  ${PY_SOURCES} ${SOURCES})
    target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

    if (COPY_PY_PATH_CONFIG)
        file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
    endif()

    

ELSE()
    message(STATUS "Python support is disabled")
    
    file(GLOB SOURCES "src/*.cpp")
    add_executable(lifelong ${SOURCES} )
    target_link_libraries(lifelong ${Boost_LIBRARIES} OpenMP::OpenMP_CXX)
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

    # build tests
    list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/driver.cpp")
    file(GLOB TEST_SOURCES tests/*.cpp)
    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SOURCE} ${SOURCES})
        target_link_libraries(${TEST_NAME}  ${Boost_LIBRARIES} OpenMP::OpenMP_CXX)
        target_compile_definitions(${TEST_NAME}  PRIVATE PYTHON=${PYTHON_FLAG})
    endforeach()
ENDIF()


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



