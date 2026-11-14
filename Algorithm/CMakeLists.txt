# Use the pinned upstream target, with no network access during configure/build.
# These options are local to this directory, not global CACHE overrides.
set(BUILD_SHARED_LIBS OFF)
set(BUILD_CLOUD_CLIENT OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_TESTS OFF)
set(BUILD_PYTHON_MODULE OFF)
set(BUILD_BENCHMARK OFF)
add_subdirectory(ruckig)

add_library(ylamp_trajectory STATIC joint_trajectory.cpp)
target_include_directories(ylamp_trajectory PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(ylamp_trajectory PRIVATE ruckig::ruckig)
target_compile_options(ylamp_trajectory PRIVATE -Wall -Wextra -Werror)
foreach(algorithm_target ruckig ylamp_trajectory)
    # Keep debug symbols, but avoid large unoptimized solver temporaries on MCU.
    # Use the same floating-point optimization in host and ARM configurations.
    target_compile_options(${algorithm_target} PRIVATE
        -O2 -fno-exceptions -fno-rtti -fno-fast-math
        -ffunction-sections -fdata-sections -fstack-usage)
endforeach()
