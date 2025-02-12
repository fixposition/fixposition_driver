# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)

project(fixposition_driver_lib VERSION 8.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith -Woverloaded-virtual")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

find_package(Boost 1.65.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fpsdk_common REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${fpsdk_common_INCLUDE_DIR}
)


# BUILD EXECUTABLE =====================================================================================================

add_library(
  ${PROJECT_NAME} SHARED
  src/fixposition_driver.cpp
  src/helper.cpp
  src/params.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${fpsdk_common_LIBRARIES} pthread)


# INSTALL ==============================================================================================================

list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION .
)

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
# See doc: https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#example-generating-package-files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
