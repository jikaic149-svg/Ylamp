cmake_minimum_required(VERSION 3.22)
project(YlampBSP LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(BSP_HOST_TEST "Build HAL-fake behavioral tests on the host" OFF)
option(YLAMP_J5_TEST_ONLY "Exclusive SWD-triggered CAN1/J5 bench firmware; no dual-axis control" OFF)
option(YLAMP_CAN_CHECK_ONLY "Exclusive five-motor query/zero-gain enable/disable check" OFF)
option(YLAMP_CAN3_TELEMETRY "Enable optional CAN3 FD telemetry alongside board USB" ON)
if(YLAMP_J5_TEST_ONLY AND YLAMP_CAN_CHECK_ONLY)
    message(FATAL_ERROR "Select only one exclusive bench firmware")
endif()
set(BSP_CUBE_ROOT "" CACHE PATH "Unmodified DM-MC02 H7_BSP-HEAD reference project")

add_subdirectory(Algorithm)

add_library(ylamp_bsp STATIC
    BSP/CAN/bsp_can.c
    BSP/CAN/bsp_can3.c
    BSP/USB/bsp_usb.c
    BSP/UART/bsp_uart.c
    BSP/RGB/bsp_rgb.c
    BSP/KEY/bsp_key.c
    BSP/POWER/bsp_power.c
    BSP/IMU/bsp_imu.c
)
target_include_directories(ylamp_bsp PUBLIC
    BSP/CAN BSP/USB BSP/UART BSP/RGB BSP/KEY BSP/POWER BSP/IMU
)
target_compile_options(ylamp_bsp PRIVATE -Wall -Wextra -Werror)
if(YLAMP_CAN_CHECK_ONLY)
    target_compile_definitions(ylamp_bsp PRIVATE YLAMP_CAN_CAPTURE_ONLY=1)
endif()

add_library(ylamp_device STATIC Device/DM_Motor/dm_motor.c Device/BMI088/bmi088.c
    Device/Telemetry/motor_snapshot.c Device/USB/usb_telemetry.c)
target_include_directories(ylamp_device PUBLIC Device/DM_Motor Device/BMI088
    Device/Telemetry Device/USB)
target_link_libraries(ylamp_device PUBLIC ylamp_bsp)
target_compile_options(ylamp_device PRIVATE -Wall -Wextra -Werror)

add_library(ylamp_model STATIC Model/robot_model.c config/robot_model.c)
target_include_directories(ylamp_model PUBLIC Model)
target_compile_options(ylamp_model PRIVATE -Wall -Wextra -Werror)
target_link_libraries(ylamp_model PUBLIC m)
add_library(ylamp_task STATIC Task/motor_task.c Mode/drag_mode.c Mode/motion_mode.c Mode/motor_mode.c Task/motor_tuning.c
    Task/motor_serial.c Task/motor_usb.c Task/motor_telemetry.c Task/can3_telemetry.c Task/joint_calibration.c
    Task/j5_test.c Task/can_check.c Mode/j5_test_mode.c config/calibration_config.c config/motor_config.c)
target_include_directories(ylamp_task PUBLIC Task Mode Model config)
target_link_libraries(ylamp_task PUBLIC ylamp_model)
target_link_libraries(ylamp_task PUBLIC ylamp_trajectory)
target_link_libraries(ylamp_task PUBLIC ylamp_device m)
target_compile_options(ylamp_task PRIVATE -Wall -Wextra -Werror)
# Keep stack evidence for the newly active planner call chain in both builds.
set_source_files_properties(Task/motor_task.c Task/motor_tuning.c Task/motor_serial.c Task/motor_usb.c
    Task/can3_telemetry.c BSP/CAN/bsp_can3.c BSP/CAN/bsp_can.c
    BSP/USB/bsp_usb.c Device/USB/usb_telemetry.c Device/Telemetry/motor_snapshot.c
    Mode/motor_mode.c Mode/motion_mode.c Task/j5_test.c Task/can_check.c App/main.c App/rtos.c
    PROPERTIES COMPILE_OPTIONS "-fstack-usage")

if(BSP_HOST_TEST)
    target_include_directories(ylamp_bsp PUBLIC tests/fakes)
    add_executable(bsp_tests tests/test_bsp.c tests/headers.cpp)
    target_link_libraries(bsp_tests PRIVATE ylamp_task)
    target_compile_options(bsp_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    enable_testing()
    add_executable(j5_test_tests tests/test_j5_test.c)
    target_link_libraries(j5_test_tests PRIVATE ylamp_task)
    target_compile_options(j5_test_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME j5_isolated_test COMMAND j5_test_tests)
    add_executable(can_check_tests tests/test_can_check.c)
    target_link_libraries(can_check_tests PRIVATE ylamp_task)
    target_compile_options(can_check_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME can_enable_check COMMAND can_check_tests)
    add_executable(usb_tests tests/test_usb.c)
    target_link_libraries(usb_tests PRIVATE ylamp_task m)
    target_compile_options(usb_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME usb_telemetry COMMAND usb_tests)
    add_test(NAME usb_six_joint COMMAND usb_tests six)
    foreach(stage 1 2 3 4)
        add_test(NAME usb_init_failure_${stage} COMMAND usb_tests init-failure ${stage})
    endforeach()
    add_executable(robot_model_tests tests/test_robot_model.c)
    target_link_libraries(robot_model_tests PRIVATE ylamp_model)
    target_compile_options(robot_model_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME robot_model_standalone COMMAND robot_model_tests)
    # C caller -> C++ adapter -> official C++ core; no HAL or RTOS dependency.
    add_executable(trajectory_tests tests/test_joint_trajectory.c tests/trajectory_alloc.cpp)
    target_link_libraries(trajectory_tests PRIVATE ylamp_trajectory ylamp_model ruckig::ruckig m)
    target_compile_options(trajectory_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    target_compile_options(trajectory_tests PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-fast-math>)
    target_link_options(trajectory_tests PRIVATE
        -Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc,--wrap=free)
    add_test(NAME joint_trajectory COMMAND trajectory_tests)
    add_executable(telemetry_tests tests/test_motor_telemetry.c Task/motor_telemetry.c)
    target_include_directories(telemetry_tests PRIVATE Task)
    target_compile_options(telemetry_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME telemetry_records COMMAND telemetry_tests)
    add_executable(calibration_tests tests/test_calibration.c
        Task/joint_calibration.c config/calibration_config.c)
    target_include_directories(calibration_tests PRIVATE Task Mode Model Algorithm config Device/BMI088 Device/DM_Motor)
    target_link_libraries(calibration_tests PRIVATE ylamp_bsp m)
    target_compile_options(calibration_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME joint_calibration COMMAND calibration_tests)
    add_test(NAME saved_joint_reference COMMAND calibration_tests saved)
    add_executable(imu_tests tests/test_imu.c Device/BMI088/bmi088.c)
    target_include_directories(imu_tests PRIVATE Device/BMI088)
    target_link_libraries(imu_tests PRIVATE ylamp_bsp m)
    target_compile_options(imu_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME imu_protocol COMMAND imu_tests)
    add_test(NAME imu_power_readback COMMAND imu_tests power-readback)
    add_test(NAME imu_power_timeout COMMAND imu_tests power-timeout)
    add_executable(rtos_tests tests/test_rtos.c App/rtos.c)
    target_include_directories(rtos_tests PRIVATE tests/fakes config App)
    target_compile_options(rtos_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME rtos_schedule COMMAND rtos_tests)
    add_executable(fault_tests tests/test_fault.c)
    target_include_directories(fault_tests PRIVATE App)
    target_compile_options(fault_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME fault_stack_frame COMMAND fault_tests)
    # Mode must link and run without Task, Device/BSP implementations or RTOS.
    add_executable(drag_mode_tests tests/test_drag_mode.c
        Mode/drag_mode.c Mode/motion_mode.c Mode/motor_mode.c config/motor_config.c)
    target_include_directories(drag_mode_tests PRIVATE
        Mode Model config Device/DM_Motor BSP/CAN tests/fakes)
    target_link_libraries(drag_mode_tests PRIVATE ylamp_model ylamp_trajectory m)
    target_compile_options(drag_mode_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME drag_mode_standalone COMMAND drag_mode_tests)
    add_executable(motion_mode_tests tests/test_motion_mode.c Mode/motion_mode.c config/motor_config.c)
    target_include_directories(motion_mode_tests PRIVATE
        Mode Model config Device/DM_Motor BSP/CAN tests/fakes)
    target_link_libraries(motion_mode_tests PRIVATE ylamp_model ylamp_trajectory m)
    target_compile_options(motion_mode_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME motion_mode_standalone COMMAND motion_mode_tests)
    add_test(NAME bsp_behavior COMMAND bsp_tests)
    add_test(NAME can_startup COMMAND bsp_tests can-startup)
    add_test(NAME can_five_axis COMMAND bsp_tests can-five-axis)
    add_test(NAME can_motor_config COMMAND bsp_tests can-motor-config)
    add_test(NAME can3_fd_telemetry COMMAND bsp_tests can3-fd)
    add_test(NAME can3_init_failure COMMAND bsp_tests can3-init-failure)
    add_test(NAME can_diagnostics COMMAND bsp_tests can-diagnostics)
    add_test(NAME can_recovery COMMAND bsp_tests can-recovery)
    add_test(NAME serial_link COMMAND bsp_tests serial-link)
    add_test(NAME serial_reliable COMMAND bsp_tests serial-reliable)
    add_test(NAME key_pressed_at_boot COMMAND bsp_tests key-boot)
    add_test(NAME dm_motor_behavior COMMAND bsp_tests dm-motor)
    add_test(NAME dm_motor_capacity COMMAND bsp_tests dm-motor-full)
    foreach(case clamp locked drag motion impedance stale sample-clock transport timing bounds arming stop-arming
                 drag-speed drag-polarity drag-dynamics grip start-still tracking tuning model partial temperature state coordinates coordinate-hold can-schedule
                 recovery recovery-motion recovery-idle recovery-hold recovery-key recovery-stop recovery-disable
                 recovery-flush recovery-stale recovery-hot recovery-limit)
        add_test(NAME motor_task_${case} COMMAND bsp_tests "task-${case}")
    endforeach()
else()
    if(NOT EXISTS "${BSP_CUBE_ROOT}/Core/Inc/main.h")
        message(FATAL_ERROR "Set BSP_CUBE_ROOT to the existing H7_BSP-HEAD directory")
    endif()
    target_compile_definitions(ylamp_bsp PUBLIC USE_HAL_DRIVER STM32H723xx)
    target_include_directories(ylamp_bsp PUBLIC
        "${BSP_CUBE_ROOT}/Core/Inc"
        "${BSP_CUBE_ROOT}/Drivers/STM32H7xx_HAL_Driver/Inc"
        "${BSP_CUBE_ROOT}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
        "${BSP_CUBE_ROOT}/Drivers/CMSIS/Include"
        "${BSP_CUBE_ROOT}/USB_DEVICE/App"
        "${BSP_CUBE_ROOT}/USB_DEVICE/Target"
        "${BSP_CUBE_ROOT}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
        "${BSP_CUBE_ROOT}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
    )

    # 复用未修改的 vendor 源码；RTOS 配置与应用调度由本项目维护。
    enable_language(ASM)
    set(HAL_DIR "${BSP_CUBE_ROOT}/Drivers/STM32H7xx_HAL_Driver/Src")
    add_library(bsp_hal_check STATIC
        "${BSP_CUBE_ROOT}/Core/Src/gpio.c"
        "${BSP_CUBE_ROOT}/Core/Src/fdcan.c"
        "${BSP_CUBE_ROOT}/Core/Src/spi.c"
        "${BSP_CUBE_ROOT}/Core/Src/usart.c"
        "${BSP_CUBE_ROOT}/Core/Src/system_stm32h7xx.c"
        "${HAL_DIR}/stm32h7xx_hal.c"
        "${HAL_DIR}/stm32h7xx_hal_cortex.c"
        "${HAL_DIR}/stm32h7xx_hal_dma.c"
        "${HAL_DIR}/stm32h7xx_hal_dma_ex.c"
        "${HAL_DIR}/stm32h7xx_hal_fdcan.c"
        "${HAL_DIR}/stm32h7xx_hal_flash.c"
        "${HAL_DIR}/stm32h7xx_hal_flash_ex.c"
        "${HAL_DIR}/stm32h7xx_hal_gpio.c"
        "${HAL_DIR}/stm32h7xx_hal_pwr.c"
        "${HAL_DIR}/stm32h7xx_hal_pwr_ex.c"
        "${HAL_DIR}/stm32h7xx_hal_pcd.c"
        "${HAL_DIR}/stm32h7xx_hal_pcd_ex.c"
        "${HAL_DIR}/stm32h7xx_ll_usb.c"
        "${HAL_DIR}/stm32h7xx_hal_rcc.c"
        "${HAL_DIR}/stm32h7xx_hal_rcc_ex.c"
        "${HAL_DIR}/stm32h7xx_hal_spi.c"
        "${HAL_DIR}/stm32h7xx_hal_spi_ex.c"
        "${HAL_DIR}/stm32h7xx_hal_tim.c"
        "${HAL_DIR}/stm32h7xx_hal_tim_ex.c"
        "${HAL_DIR}/stm32h7xx_hal_uart.c"
        "${HAL_DIR}/stm32h7xx_hal_uart_ex.c"
        "${BSP_CUBE_ROOT}/USB_DEVICE/Target/usbd_conf.c"
        "${BSP_CUBE_ROOT}/USB_DEVICE/App/usbd_desc.c"
        "${BSP_CUBE_ROOT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
        "${BSP_CUBE_ROOT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
        "${BSP_CUBE_ROOT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
        "${BSP_CUBE_ROOT}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
    )
    target_link_libraries(bsp_hal_check PRIVATE ylamp_bsp)
    add_executable(bsp_link_check
        tests/arm_link_check.c tests/headers.cpp
        "${BSP_CUBE_ROOT}/startup_stm32h723xx.s"
    )
    # whole-archive 防止未调用的 BSP 文件被静态库抽取规则漏检。
    # Link the core file here without imposing its C++20 headers on SDK checks.
    target_link_libraries(bsp_link_check PRIVATE
        -Wl,--whole-archive ylamp_task ylamp_device ylamp_bsp ylamp_trajectory
        $<TARGET_FILE:ruckig>
        -Wl,--no-whole-archive bsp_hal_check m
    )
    # CubeMX 的 READONLY 段语法不被本机 GCC10 配套 ld 支持。
    # 只在 build 内生成兼容副本，不修改参考工程的内存布局或源文件。
    file(READ "${BSP_CUBE_ROOT}/STM32H723XG_FLASH.ld" BSP_LINK_SCRIPT)
    if(CMAKE_C_COMPILER_VERSION VERSION_LESS 11)
        string(REPLACE "(READONLY)" "" BSP_LINK_SCRIPT "${BSP_LINK_SCRIPT}")
    endif()
    # Current MPU marks AXI SRAM's first 64KiB non-cacheable. Keep DMA buffers
    # there, not in the SDK's default .bss (DTCM is inaccessible to DMA1/2).
    string(REPLACE "SECTIONS\n{" "SECTIONS\n{\n  .dma_buffer (NOLOAD) : ALIGN(32)\n  {\n    *(.dma_buffer*)\n    . = ALIGN(32);\n  } >RAM_D1\n  ASSERT(ADDR(.dma_buffer) + SIZEOF(.dma_buffer) <= 0x24010000, \"DMA exceeds MPU non-cacheable region\")"
        BSP_LINK_SCRIPT "${BSP_LINK_SCRIPT}")
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/bsp_link_check.ld"
        CONTENT "${BSP_LINK_SCRIPT}")
    target_link_options(bsp_link_check PRIVATE
        "-T${CMAKE_CURRENT_BINARY_DIR}/bsp_link_check.ld"
        --specs=nano.specs --specs=nosys.specs
        -Wl,--no-gc-sections
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/bsp_link_check.map"
        -Wl,--print-memory-usage
    )
    set_target_properties(bsp_link_check PROPERTIES SUFFIX ".elf")

    set(FREERTOS_DIR "${BSP_CUBE_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")
    add_library(ylamp_freertos STATIC
        "${FREERTOS_DIR}/tasks.c"
        "${FREERTOS_DIR}/queue.c"
        "${FREERTOS_DIR}/list.c"
        "${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c"
    )
    # Keep this directory BEFORE Cube Core/Inc's unrelated FreeRTOSConfig.h.
    target_include_directories(ylamp_freertos BEFORE PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/config"
        "${CMAKE_CURRENT_SOURCE_DIR}/App"
        "${FREERTOS_DIR}/include"
        "${FREERTOS_DIR}/portable/GCC/ARM_CM4F"
    )
    target_compile_options(ylamp_freertos PRIVATE -Wall -Wextra)

    # Native CM4F port supports Cortex-M7 r1+; H723 header reports r1p2.
    # No CMSIS-RTOS2, heap, timer task, SEGGER hooks or old reference tasks.
    add_executable(ylamp_firmware
        App/main.c App/rtos.c App/fault.c App/serial_dma.c
        "${BSP_CUBE_ROOT}/Core/Src/stm32h7xx_hal_msp.c"
        "${BSP_CUBE_ROOT}/Core/Src/stm32h7xx_hal_timebase_tim.c"
        "${BSP_CUBE_ROOT}/startup_stm32h723xx.s"
    )
    # USB BSP introduces callbacks back into the vendor stack. Explicit group
    # rescanning resolves both directions without whole-archiving production.
    target_link_libraries(ylamp_firmware PRIVATE -Wl,--start-group
        ylamp_freertos ylamp_task ylamp_device ylamp_bsp bsp_hal_check
        -Wl,--end-group m)
    target_compile_options(ylamp_firmware PRIVATE -Wall -Wextra -Werror)
    if(YLAMP_CAN3_TELEMETRY)
        target_compile_definitions(ylamp_firmware PRIVATE YLAMP_CAN3_TELEMETRY=1)
    endif()
    if(YLAMP_J5_TEST_ONLY)
        target_compile_definitions(ylamp_firmware PRIVATE YLAMP_J5_TEST_ONLY=1)
    endif()
    if(YLAMP_CAN_CHECK_ONLY)
        target_compile_definitions(ylamp_firmware PRIVATE YLAMP_CAN_CHECK_ONLY=1)
    endif()
    set_source_files_properties(App/fault.c PROPERTIES
        COMPILE_OPTIONS "-mgeneral-regs-only;-fstack-usage")
    target_link_options(ylamp_firmware PRIVATE
        "-T${CMAKE_CURRENT_BINARY_DIR}/bsp_link_check.ld"
        --specs=nano.specs --specs=nosys.specs
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/ylamp_firmware.map"
        -Wl,--print-memory-usage
    )
    set_target_properties(ylamp_firmware PROPERTIES SUFFIX ".elf")
    add_custom_command(TARGET ylamp_firmware POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O ihex
            "$<TARGET_FILE:ylamp_firmware>"
            "${CMAKE_CURRENT_BINARY_DIR}/ylamp_firmware.hex"
        COMMAND "${CMAKE_OBJCOPY}" -O binary
            "$<TARGET_FILE:ylamp_firmware>"
            "${CMAKE_CURRENT_BINARY_DIR}/ylamp_firmware.bin"
        VERBATIM
    )
endif()
