cmake_minimum_required(VERSION 3.28)

project(
        spline
        VERSION 0.0.1
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Werror -Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/r3_spline.cpp
        src/utilities.cpp
)
add_library(${PROJECT_NAME} SHARED
        ${SRC_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
        $<INSTALL_INTERFACE:include/>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        Eigen3::Eigen
)

message(STATUS "Starting to FetchContent googletest - this requires a network connection. ")
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
enable_testing()
include(GoogleTest)

set(TESTS
        src/utilities.test.cpp
        src/r3_spline.test.cpp
)
foreach (TEST IN LISTS TESTS)
    get_filename_component(TEST_NAME ${TEST} NAME_WLE)
    add_executable(${TEST_NAME} ${TEST})
    target_link_libraries(${TEST_NAME}
            GTest::gtest_main
            ${PROJECT_NAME}
    )
    gtest_discover_tests(${TEST_NAME})
endforeach ()
