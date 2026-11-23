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
  add_library(joint_controller_doctest INTERFACE)
  target_include_directories(joint_controller_doctest SYSTEM INTERFACE "${doctest_SOURCE_DIR}")
  add_library(doctest::doctest ALIAS joint_controller_doctest)
endif()

add_library(joint_controller_test_main OBJECT doctest_main.cpp)
target_link_libraries(joint_controller_test_main PUBLIC doctest::doctest)

# Core (no franky): planner, law, params and the randomized soak.
foreach(name params planner law soak)
  add_executable(test_${name} test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE
    joint_controller::core joint_controller_test_main joint_controller_warnings)
  add_test(NAME ${name} COMMAND test_${name})
endforeach()

if(JOINT_CONTROLLER_WITH_FRANKY)
  add_executable(test_motion test_motion.cpp)
  target_link_libraries(test_motion PRIVATE
    joint_controller::motion joint_controller_test_main joint_controller_warnings)
  add_test(NAME motion COMMAND test_motion)
endif()
