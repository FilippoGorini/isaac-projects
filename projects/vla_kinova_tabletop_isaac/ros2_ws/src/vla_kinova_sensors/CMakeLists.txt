cmake_minimum_required(VERSION 3.8)
project(vla_kinova_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# vision_focus: fixes the wrist colour camera focus through the Kortex API (same binary API as kortex_driver)
# ponytail: x86_64 only, copy kortex_driver's aarch64 URL switch if this ever builds on ARM
include(FetchContent)
FetchContent_Declare(kinova_binary_api
  URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/linux_x86-64_gcc_5.4.zip)
FetchContent_MakeAvailable(kinova_binary_api)
set(K ${kinova_binary_api_SOURCE_DIR})
add_executable(vision_focus src/vision_focus.cpp)
target_compile_definitions(vision_focus PRIVATE _OS_UNIX)
target_compile_options(vision_focus PRIVATE -w)  # Kortex/protobuf headers are noisy under -Wpedantic
target_include_directories(vision_focus SYSTEM PRIVATE
  ${K}/include ${K}/include/client ${K}/include/client_stubs ${K}/include/messages ${K}/include/common)
target_link_libraries(vision_focus ${K}/lib/release/libKortexApiCpp.a pthread)
install(TARGETS vision_focus DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/focus_after_stream.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
