########################################################################
# Project setup -- only needed if device support is a stand-alone build
# We recommend that the support module be built in-tree with the driver.
########################################################################
cmake_minimum_required(VERSION 3.3...3.10)
project(SoapySX CXX)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

########################################################################
# Header and library resources needed to communicate with the device.
# These may be found within the build tree or in an external project.
########################################################################
set(MY_DEVICE_INCLUDE_DIRS "")
set(MY_DEVICE_LIBRARIES "asound")

########################################################################
# Source files
########################################################################
set(MY_SOURCE_FILES SoapySX.cpp)

########################################################################
# Add version
########################################################################
set(VERSION_SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/version.cpp")
set(VERSION_SCRIPT "${CMAKE_SOURCE_DIR}/version.sh")

find_package(Git REQUIRED)

# Resolve the index through Git so this also works when sxxcvr is checked out
# as a submodule (where .git is a pointer file rather than a directory).
execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${CMAKE_SOURCE_DIR}" rev-parse --git-path index
    OUTPUT_VARIABLE GIT_INDEX_FILE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE GIT_INDEX_RESULT
)
set(VERSION_DEPENDS ${VERSION_SCRIPT} ${MY_SOURCE_FILES})
if(GIT_INDEX_RESULT EQUAL 0)
    get_filename_component(
        GIT_INDEX_FILE "${GIT_INDEX_FILE}" ABSOLUTE
        BASE_DIR "${CMAKE_SOURCE_DIR}"
    )
    if(EXISTS "${GIT_INDEX_FILE}")
        list(APPEND VERSION_DEPENDS "${GIT_INDEX_FILE}")
    endif()
endif()

add_custom_command(
    OUTPUT ${VERSION_SOURCE_FILE}
    COMMAND "${VERSION_SCRIPT}" "${GIT_EXECUTABLE}" > "${VERSION_SOURCE_FILE}"
    DEPENDS ${VERSION_DEPENDS}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

########################################################################
# build the module
########################################################################
find_package(SoapySDR CONFIG)

if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()

include_directories(${MY_DEVICE_INCLUDE_DIRS})
SOAPY_SDR_MODULE_UTIL(
    TARGET SXSupport
    SOURCES ${MY_SOURCE_FILES} ${VERSION_SOURCE_FILE}
    LIBRARIES ${MY_DEVICE_LIBRARIES}
)

########################################################################
# Other files to install
########################################################################

option(INSTALL_PIPEWIRE_CONF "Install PipeWire configuration to prevent it from trying to use SX1255 as a normal sound card" ON)
if (INSTALL_PIPEWIRE_CONF)
install(FILES 60-pipewire-do-not-use-i2s.lua DESTINATION /usr/share/wireplumber/main.lua.d/)
endif (INSTALL_PIPEWIRE_CONF)
