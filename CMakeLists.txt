cmake_minimum_required(VERSION 3.11)
project(PSMoveLSL VERSION 0.1.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
IF (NOT MSVC)
    set(CMAKE_CXX_FLAGS "-std=c++11")
    IF (NOT CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    ENDIF()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-switch")
ENDIF()

# Disable GLM warnings in Darwin
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #warning: 'register' storage class specifier is deprecated and incompatible with C++1z
    #warning: operator '<<' has lower precedence than '-'; '-' will be evaluated first
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register -Wno-shift-op-parentheses")
ENDIF()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Third Party

# LSL should either be found because this project is included in a larger cmake project
# that includes LSL, or because its in a sister directory or in the build_deps directory.
if(NOT TARGET LSL::lsl)
    # when building out of tree LSL_ROOT needs to be specified on the cmd line
    file(TO_CMAKE_PATH "${LSL_INSTALL_ROOT}" LSL_INSTALL_ROOT)
    list(APPEND LSL_INSTALL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../LSL/liblsl/build/install")
    find_package(LSL
        HINTS
            ${LSL_INSTALL_ROOT}/share/LSL/
            ${LSL_INSTALL_ROOT}/LSL/share/LSL
            ${ROOT_DIR}/build_deps/LSL/share/LSL
        QUIET)
    if(NOT LSL_FOUND)
        message(FATAL_ERROR "Precompiled LSL was not found. See https://github.com/labstreaminglayer/labstreaminglayer/blob/master/doc/BUILD.md#lsl_install_root for more information.")
    endif()
    list(APPEND CMAKE_MODULE_PATH ${LSL_DIR})
    message(STATUS "Looking for LSLCMake in ${LSL_DIR}")
    include(LSLCMake)
endif()

# Qt should be installed with a package manager like vcpkg, homebrew, or apt-get
find_package(Qt5 REQUIRED COMPONENTS Core Xml Gui Widgets)

# PSMoveClient_CAPI is downloaded into build_deps
set(PSM_ROOT_DIR ${ROOT_DIR}/build_deps/PSMoveService)
find_package(PSMoveService REQUIRED)
# Use with ${PSM_INCLUDE_DIR} and ${PSM_LIBRARIES}

# Configure the cmake for the driver
add_subdirectory(src)
