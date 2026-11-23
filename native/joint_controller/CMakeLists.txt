cmake_minimum_required(VERSION 3.24)

# Single source of truth for the version: ./VERSION (also read by pyproject.toml).
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" JOINT_CONTROLLER_VERSION LIMIT_COUNT 1)
project(joint_controller VERSION ${JOINT_CONTROLLER_VERSION} LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(DEFINED SKBUILD)
  set(_jc_default_python ON)
  set(_jc_default_tests OFF)
else()
  set(_jc_default_python OFF)
  set(_jc_default_tests ${PROJECT_IS_TOP_LEVEL})
endif()

option(JOINT_CONTROLLER_WITH_FRANKY "Build TrajectoryImpedanceMotion (needs franky headers and libfranky.so)" ON)
option(JOINT_CONTROLLER_BUILD_PYTHON "Build the pybind11 extension joint_controller._joint_controller"
       ${_jc_default_python})
option(JOINT_CONTROLLER_BUILD_TESTS "Build the C++ tests (CTest)" ${_jc_default_tests})

# franky does not install a CMake package, so it is described by hand. Build against
# the franky that will be deployed: its headers come from the franky source tree of the
# same tag, its libfranky.so from the installed franky package. Two layouts are supported:
#   wheel:  an auditwheel-repaired franky wheel (docker/build-wheels.sh); libfranka is the
#           wheel's bundled site-packages/franky_control.libs/libfranka-<hash>.so.<ver>,
#           passed in FRANKY_LINK_LIBRARIES;
#   source: franky pip-installed from source against a system libfranka (scripts/
#           install-on-ubuntu.sh); FRANKY_LINK_LIBRARIES is empty and Franka::Franka (the
#           libfranka franky was built against) is linked.
set(FRANKY_INCLUDE_DIR "" CACHE PATH "franky source include/ directory (same tag as the installed franky)")
set(FRANKY_LIBRARY "" CACHE FILEPATH "libfranky.so of the installed franky (site-packages/franky/)")
set(FRANKY_LINK_LIBRARIES "" CACHE STRING
    "Further libraries to link instead of Franka::Franka, e.g. the wheel's bundled libfranka-<hash>.so.<ver>")
set(JOINT_CONTROLLER_INSTALL_RPATH "$ORIGIN/../franky;$ORIGIN/../franky_control.libs" CACHE STRING
    "RPATH of the Python extension (finds franky's libraries next to it in site-packages; the source layout also gets libfranka's directory)")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ---------------------------------------------------------------------------
# warnings (applied to our own targets only)
add_library(joint_controller_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(joint_controller_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

# Build identity of the franky wheel this build links against (filled in by
# docker/build-wheels.sh). Python reads it from joint_controller/build_info.json to
# check, without importing either extension, that the installed franky wheel is
# the one from the same build (see README, "ABI coupling rule").
set(JOINT_CONTROLLER_FRANKY_DIST_VERSION "" CACHE STRING "Version of the franky_control wheel built alongside")
set(JOINT_CONTROLLER_FRANKY_COMMIT "" CACHE STRING "franky git commit the franky wheel and headers come from")
set(JOINT_CONTROLLER_BUILD_ID "" CACHE STRING "Identifier of the build that produced both wheels")

# ---------------------------------------------------------------------------
# core: the franky-independent planner and control law (Eigen only)
add_library(joint_controller_core STATIC src/params.cpp src/planner.cpp src/law.cpp)
add_library(joint_controller::core ALIAS joint_controller_core)
target_compile_features(joint_controller_core PUBLIC cxx_std_17)
target_include_directories(joint_controller_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated/include>)
target_link_libraries(joint_controller_core PUBLIC Eigen3::Eigen PRIVATE joint_controller_warnings)
set_target_properties(joint_controller_core PROPERTIES
  CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)

# ---------------------------------------------------------------------------
# motion: franky::Motion<franka::Torques> wrapper (TrajectoryImpedanceMotion)
if(JOINT_CONTROLLER_WITH_FRANKY)
  if(NOT FRANKY_INCLUDE_DIR OR NOT EXISTS "${FRANKY_INCLUDE_DIR}/franky/motion/motion.hpp")
    message(FATAL_ERROR "JOINT_CONTROLLER_WITH_FRANKY=ON needs FRANKY_INCLUDE_DIR=<franky>/include")
  endif()
  if(NOT FRANKY_LIBRARY OR NOT EXISTS "${FRANKY_LIBRARY}")
    message(FATAL_ERROR "JOINT_CONTROLLER_WITH_FRANKY=ON needs FRANKY_LIBRARY=<site-packages>/franky/libfranky.so")
  endif()
  find_package(Threads REQUIRED)
  find_package(Franka 0.8 REQUIRED)  # headers (and, without FRANKY_LINK_LIBRARIES, the library)
  message(STATUS "joint_controller: libfranka ${Franka_VERSION}, franky ${FRANKY_LIBRARY}")

  # franky's class layouts depend on these defines (e.g. RobotState under FRANKA_0_8,
  # Robot under FRANKA_0_15): replicate franky's own CMakeLists.txt exactly.
  set(JOINT_CONTROLLER_FRANKA_DEFINES "")
  foreach(franka_min_version 0.8 0.9 0.10 0.15 0.18)
    if(${Franka_VERSION} VERSION_GREATER_EQUAL "${franka_min_version}.0")
      string(REPLACE "." "_" franka_min_version_str "${franka_min_version}")
      list(APPEND JOINT_CONTROLLER_FRANKA_DEFINES "FRANKA_${franka_min_version_str}")
    endif()
  endforeach()

  add_library(joint_controller_franky SHARED IMPORTED GLOBAL)
  add_library(joint_controller::franky ALIAS joint_controller_franky)
  get_target_property(_franka_includes Franka::Franka INTERFACE_INCLUDE_DIRECTORIES)
  if(FRANKY_LINK_LIBRARIES)
    set(_franky_deps ${FRANKY_LINK_LIBRARIES})
  else()
    set(_franky_deps Franka::Franka)
  endif()
  set_target_properties(joint_controller_franky PROPERTIES
    IMPORTED_LOCATION "${FRANKY_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${FRANKY_INCLUDE_DIR};${_franka_includes}"
    INTERFACE_COMPILE_DEFINITIONS "${JOINT_CONTROLLER_FRANKA_DEFINES}"
    INTERFACE_LINK_LIBRARIES "${_franky_deps};Eigen3::Eigen;Threads::Threads")
  find_package(ruckig QUIET)  # only needed if franky headers pull in ruckig
  if(ruckig_FOUND)
    get_target_property(_ruckig_includes ruckig::ruckig INTERFACE_INCLUDE_DIRECTORIES)
    set_property(TARGET joint_controller_franky APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${_ruckig_includes})
  endif()

  add_library(joint_controller_motion STATIC src/motion.cpp)
  add_library(joint_controller::motion ALIAS joint_controller_motion)
  target_link_libraries(joint_controller_motion
    PUBLIC joint_controller::core joint_controller::franky
    PRIVATE joint_controller_warnings)
  target_compile_definitions(joint_controller_motion PUBLIC JOINT_CONTROLLER_WITH_FRANKY)
  set_target_properties(joint_controller_motion PROPERTIES
    CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
endif()

# ---------------------------------------------------------------------------
# version header + build_info.json
set(JOINT_CONTROLLER_LIBFRANKY_SHA256 "")
set(JOINT_CONTROLLER_LIBFRANKA_FILE "")
set(JOINT_CONTROLLER_LIBFRANKA_VERSION "")
set(JOINT_CONTROLLER_FRANKY_LAYOUT "")
set(JOINT_CONTROLLER_LIBFRANKA_PATH "")
set(JOINT_CONTROLLER_LIBFRANKA_SHA256 "")
set(_jc_extra_rpath "")
if(JOINT_CONTROLLER_WITH_FRANKY)
  file(SHA256 "${FRANKY_LIBRARY}" JOINT_CONTROLLER_LIBFRANKY_SHA256)
  set(JOINT_CONTROLLER_LIBFRANKA_VERSION "${Franka_VERSION}")
  foreach(_lib IN LISTS FRANKY_LINK_LIBRARIES)
    get_filename_component(_name "${_lib}" NAME)
    if(_name MATCHES "^libfranka")
      set(JOINT_CONTROLLER_LIBFRANKA_FILE "${_name}")
    endif()
  endforeach()
  if(JOINT_CONTROLLER_LIBFRANKA_FILE)
    set(JOINT_CONTROLLER_FRANKY_LAYOUT "wheel")
  else()
    # Source layout: record the system libfranka franky and this extension link against, so
    # `rwrl doctor` notices when it is replaced (another libfranka install) after the build.
    set(JOINT_CONTROLLER_FRANKY_LAYOUT "source")
    get_target_property(_franka_location Franka::Franka LOCATION)
    if(NOT _franka_location OR NOT EXISTS "${_franka_location}")
      message(FATAL_ERROR "joint_controller: cannot resolve the libfranka library of Franka::Franka")
    endif()
    file(REAL_PATH "${_franka_location}" JOINT_CONTROLLER_LIBFRANKA_PATH)
    file(SHA256 "${JOINT_CONTROLLER_LIBFRANKA_PATH}" JOINT_CONTROLLER_LIBFRANKA_SHA256)
    get_filename_component(_jc_extra_rpath "${JOINT_CONTROLLER_LIBFRANKA_PATH}" DIRECTORY)
    message(STATUS "joint_controller: source layout, libfranka ${JOINT_CONTROLLER_LIBFRANKA_PATH}")
  endif()
endif()
if(JOINT_CONTROLLER_WITH_FRANKY)
  set(JOINT_CONTROLLER_WITH_FRANKY_JSON true)
else()
  set(JOINT_CONTROLLER_WITH_FRANKY_JSON false)
endif()
configure_file(include/joint_controller/version.hpp.in
               "${CMAKE_CURRENT_BINARY_DIR}/generated/include/joint_controller/version.hpp" @ONLY)
configure_file(python/build_info.json.in "${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.json" @ONLY)

# ---------------------------------------------------------------------------
# Python extension
if(JOINT_CONTROLLER_BUILD_PYTHON)
  find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)
  message(STATUS "joint_controller: pybind11 ${pybind11_VERSION}")
  pybind11_add_module(_joint_controller MODULE python/bindings.cpp)
  if(JOINT_CONTROLLER_WITH_FRANKY)
    target_link_libraries(_joint_controller PRIVATE joint_controller::motion joint_controller_warnings)
  else()
    target_link_libraries(_joint_controller PRIVATE joint_controller::core joint_controller_warnings)
  endif()
  set_target_properties(_joint_controller PROPERTIES
    CXX_EXTENSIONS OFF
    INSTALL_RPATH "${JOINT_CONTROLLER_INSTALL_RPATH};${_jc_extra_rpath}"
    BUILD_WITH_INSTALL_RPATH OFF)
  install(TARGETS _joint_controller LIBRARY DESTINATION joint_controller COMPONENT python)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.json"
          DESTINATION joint_controller COMPONENT python)
endif()

# ---------------------------------------------------------------------------
if(JOINT_CONTROLLER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests/cpp)
endif()
