########################################################################
# Project setup -- only needed if device support is a stand-alone build
# We recommend that the support module be built in-tree with the driver.
########################################################################
cmake_minimum_required(VERSION 3.3...3.10)
project(SoapySX CXX)
enable_testing()

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
set(VERSION_SOURCE_FILE version.cpp)
set(VERSION_SCRIPT "${CMAKE_SOURCE_DIR}/version.sh")

find_package(Git REQUIRED)

add_custom_command(
    OUTPUT ${VERSION_SOURCE_FILE}
    COMMAND "${VERSION_SCRIPT}" "${GIT_EXECUTABLE}" > "${VERSION_SOURCE_FILE}"
    DEPENDS ../.git/index ${VERSION_SCRIPT} ${MY_SOURCE_FILES}
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
