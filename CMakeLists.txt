cmake_minimum_required(VERSION 3.22)
project(YlampBSP LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(BSP_HOST_TEST "Build HAL-fake behavioral tests on the host" OFF)
option(YLAMP_J3_REVERSE_TEST "Temporary supported J3 gravity-direction comparison" OFF)
set(ylamp_image ylamp_v12_five_axis)
foreach(retired YLAMP_J5_TEST_ONLY YLAMP_CAN_CHECK_ONLY YLAMP_LEGACY_DUAL_AXIS YLAMP_FIVE_MIT_PREVIEW YLAMP_J25_SHADOW)
    if(${retired})
        message(FATAL_ERROR "Retired ${retired} image; start with an empty five-axis build directory.")
    endif()
endforeach()
set(BSP_CUBE_ROOT "" CACHE PATH "Unmodified DM-MC02 H7_BSP-HEAD reference project")

add_library(ylamp_bsp STATIC
    BSP/CAN/bsp_can.c
    BSP/USB/bsp_usb.c
    BSP/UART/bsp_uart.c
    BSP/RGB/bsp_rgb.c
    BSP/KEY/bsp_key.c
    BSP/POWER/bsp_power.c
)
target_include_directories(ylamp_bsp PUBLIC
    BSP/CAN BSP/USB BSP/UART BSP/RGB BSP/KEY BSP/POWER
)
target_compile_options(ylamp_bsp PRIVATE -Wall -Wextra -Werror)
add_library(ylamp_device STATIC Device/DM_Motor/Common/dm_motor.c
    Device/IMU/BMI088/bmi088.c Device/IMU/BMI088/bmi088_port.c
    Device/IMU/yummyIMU/yummy_imu.c Device/Telemetry/motor_snapshot.c Device/USB/usb_telemetry.c)
target_include_directories(ylamp_device PUBLIC Device/DM_Motor/Common Device/DM_Motor Device/IMU/BMI088
    Device/IMU/yummyIMU Device/Telemetry Device/USB)
target_link_libraries(ylamp_device PUBLIC ylamp_bsp)
target_compile_options(ylamp_device PRIVATE -Wall -Wextra -Werror)

add_library(ylamp_model STATIC Model/five_workspace.c Model/five_dynamics.c)
target_include_directories(ylamp_model PUBLIC Model PRIVATE config)
target_compile_options(ylamp_model PRIVATE -Wall -Wextra -Werror)
target_link_libraries(ylamp_model PUBLIC m)
add_library(ylamp_task STATIC Task/five_control.c Task/five_mit_send.c Mode/five_mit.c Mode/five_support.c
    config/five_mit.c config/motor_config.c config/calibration_config.c
    Task/joint_calibration.c Task/motor_usb.c Task/motor_telemetry.c)
target_include_directories(ylamp_task PUBLIC Task Mode Model config)
target_link_libraries(ylamp_task PUBLIC ylamp_device ylamp_model m)
target_compile_options(ylamp_task PRIVATE -Wall -Wextra -Werror -fstack-usage)
if(YLAMP_J3_REVERSE_TEST)
    target_compile_definitions(ylamp_task PUBLIC YLAMP_J3_REVERSE_TEST=1)
endif()
if(BSP_HOST_TEST)
    enable_testing()
    target_compile_definitions(ylamp_task PUBLIC YLAMP_HOST_FIXTURES=1)
    add_executable(bsp_tests tests/test_bsp.c)
    target_link_libraries(bsp_tests PRIVATE ylamp_task)
    target_compile_options(bsp_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME bsp_behavior COMMAND bsp_tests)
    if(YLAMP_J3_REVERSE_TEST)
        add_test(NAME five-j3-reverse COMMAND bsp_tests five-j3-reverse)
    endif()
    foreach(suite can-startup can-start-failure can-motor-config can-five-axis five-mit five-control five-key five-motion-admission five-smooth-control five-cold five-cold-silent
                  five-j3-read five-j3-read-timeout five-j3-read-cancel five-j3-read-tx
                  can-diagnostics can-recovery key-boot dm-motor dm-motor-full)
        add_test(NAME ${suite} COMMAND bsp_tests ${suite})
    endforeach()
    target_include_directories(ylamp_bsp PUBLIC tests/fakes)
    add_executable(telemetry_tests tests/test_motor_telemetry.c Task/motor_telemetry.c)
    target_include_directories(telemetry_tests PRIVATE Task)
    target_compile_options(telemetry_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME telemetry_records COMMAND telemetry_tests)
    add_executable(joint_reference_tests tests/test_joint_references.c config/motor_config.c)
    target_include_directories(joint_reference_tests PRIVATE
        Model config Device/DM_Motor/Common Device/DM_Motor BSP/CAN tests/fakes)
    target_compile_options(joint_reference_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    target_link_libraries(joint_reference_tests PRIVATE m)
    add_test(NAME joint_references COMMAND joint_reference_tests)
    add_executable(imu_tests tests/test_imu.c Device/IMU/BMI088/bmi088.c
        Device/IMU/BMI088/bmi088_port.c)
    target_include_directories(imu_tests PRIVATE Device/IMU/BMI088)
    target_link_libraries(imu_tests PRIVATE ylamp_bsp m)
    target_compile_options(imu_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME imu_protocol COMMAND imu_tests)
    add_test(NAME imu_power_readback COMMAND imu_tests power-readback)
    add_test(NAME imu_power_timeout COMMAND imu_tests power-timeout)
    add_executable(yummy_imu_tests tests/test_yummy_imu.c Device/IMU/yummyIMU/yummy_imu.c)
    target_include_directories(yummy_imu_tests PRIVATE Device/IMU/yummyIMU BSP/UART tests/fakes)
    target_compile_options(yummy_imu_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    target_link_libraries(yummy_imu_tests PRIVATE m)
    add_test(NAME yummy_imu_use COMMAND yummy_imu_tests)
    add_test(NAME yummy_imu_binary COMMAND yummy_imu_tests binary)
    add_executable(rtos_tests tests/test_rtos.c App/rtos.c)
    target_include_directories(rtos_tests PRIVATE tests/fakes config App)
    target_compile_options(rtos_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME rtos_schedule COMMAND rtos_tests)
    add_executable(fault_tests tests/test_fault.c)
    target_include_directories(fault_tests PRIVATE App)
    target_compile_options(fault_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME fault_stack_frame COMMAND fault_tests)
    add_executable(usb_focus_tests tests/test_usb_focus.c)
    target_link_libraries(usb_focus_tests PRIVATE ylamp_task)
    target_compile_options(usb_focus_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME usb_focus COMMAND usb_focus_tests)
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
    # Keep the SDK unchanged, but do not carry unused bus initializers/handles
    # into the image merely because its fdcan.c also supplies CAN1 and CAN2.
    set_source_files_properties("${BSP_CUBE_ROOT}/Core/Src/fdcan.c" PROPERTIES
        COMPILE_OPTIONS "-ffunction-sections;-fdata-sections")
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
    set_source_files_properties(App/fault.c PROPERTIES
        COMPILE_OPTIONS "-mgeneral-regs-only;-fstack-usage")
    target_link_options(ylamp_firmware PRIVATE
        "-T${CMAKE_CURRENT_BINARY_DIR}/bsp_link_check.ld"
        --specs=nano.specs --specs=nosys.specs
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${ylamp_image}.map"
        -Wl,--gc-sections
        -Wl,--print-memory-usage
    )
    set_target_properties(ylamp_firmware PROPERTIES OUTPUT_NAME "${ylamp_image}" SUFFIX ".elf")
    add_custom_command(TARGET ylamp_firmware POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O ihex
            "$<TARGET_FILE:ylamp_firmware>"
            "${CMAKE_CURRENT_BINARY_DIR}/${ylamp_image}.hex"
        COMMAND "${CMAKE_OBJCOPY}" -O binary
            "$<TARGET_FILE:ylamp_firmware>"
            "${CMAKE_CURRENT_BINARY_DIR}/${ylamp_image}.bin"
        VERBATIM
    )
endif()
