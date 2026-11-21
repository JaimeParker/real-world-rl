include(FetchContent)
# Header-only use of doctest: SOURCE_SUBDIR points at a directory that does not
# exist, so doctest's own CMakeLists.txt (cmake_minimum_required < 3.5, rejected by
# CMake 4) is never configured.
FetchContent_Declare(
  doctest
  URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
  URL_HASH SHA256=632ed2c05a7f53fa961381497bf8069093f0d6628c5f26286161fbd32a560186
  SOURCE_SUBDIR no-cmake
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(doctest)
if(NOT TARGET doctest::doctest)
  add_library(serl_controller_doctest INTERFACE)
  target_include_directories(serl_controller_doctest SYSTEM INTERFACE "${doctest_SOURCE_DIR}")
  add_library(doctest::doctest ALIAS serl_controller_doctest)
endif()

add_library(serl_controller_test_main OBJECT doctest_main.cpp)
target_link_libraries(serl_controller_test_main PUBLIC doctest::doctest)

# The ORIGINAL SERL controller (vendored verbatim, see golden/serl_original/SHA256SUMS)
# compiled against ROS stand-ins. Same compiler flags as serl_controller_core, so the
# comparison is like for like.
add_library(serl_golden_reference STATIC
  golden/serl_original/src/cartesian_impedance_controller.cpp
  golden/serl_original/src/joint_position_controller.cpp
  golden/serl_reference.cpp
  golden/serl_joint_reference.cpp)
target_include_directories(serl_golden_reference
  PRIVATE golden/ros_stubs golden/serl_original/include golden/ros_control_original/include
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(serl_golden_reference PRIVATE
  "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/golden/ros_stubs/boost_bind_shim.h")
target_link_libraries(serl_golden_reference PUBLIC serl_controller::core)
set_target_properties(serl_golden_reference PROPERTIES CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON)

add_executable(test_golden_equivalence test_golden_equivalence.cpp)
target_link_libraries(test_golden_equivalence PRIVATE
  serl_golden_reference serl_controller::core serl_controller_test_main serl_controller_warnings)
target_compile_definitions(test_golden_equivalence PRIVATE
  SERL_ORIGINAL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden/serl_original")
add_test(NAME golden_equivalence COMMAND test_golden_equivalence)

add_executable(test_joint_reset_golden test_joint_reset_golden.cpp)
target_link_libraries(test_joint_reset_golden PRIVATE
  serl_golden_reference serl_controller::core serl_controller_test_main serl_controller_warnings)
target_compile_definitions(test_joint_reset_golden PRIVATE
  FRANKA_ROS_ORIGINAL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden/franka_ros_original")
add_test(NAME joint_reset_golden COMMAND test_joint_reset_golden)

if(SERL_CONTROLLER_WITH_FRANKY)
  add_executable(test_motion test_motion.cpp)
  target_link_libraries(test_motion PRIVATE
    serl_controller::motion serl_controller_test_main serl_controller_warnings)
  add_test(NAME motion COMMAND test_motion)
endif()
