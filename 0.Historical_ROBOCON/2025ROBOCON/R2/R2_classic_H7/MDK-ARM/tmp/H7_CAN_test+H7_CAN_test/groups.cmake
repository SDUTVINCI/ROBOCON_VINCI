# groups.cmake

# group Application/MDK-ARM
add_library(Group_Application_MDK-ARM OBJECT
  "${SOLUTION_ROOT}/startup_stm32h723xx.s"
)
target_include_directories(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_MDK-ARM PUBLIC
  Group_Application_MDK-ARM_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  STM32H723xx
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/startup_stm32h723xx.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)

# group Application/User/Core
add_library(Group_Application_User_Core OBJECT
  "${SOLUTION_ROOT}/../Core/Src/main.c"
  "${SOLUTION_ROOT}/../Core/Src/gpio.c"
  "${SOLUTION_ROOT}/../Core/Src/freertos.c"
  "${SOLUTION_ROOT}/../Core/Src/dma.c"
  "${SOLUTION_ROOT}/../Core/Src/fdcan.c"
  "${SOLUTION_ROOT}/../Core/Src/i2c.c"
  "${SOLUTION_ROOT}/../Core/Src/memorymap.c"
  "${SOLUTION_ROOT}/../Core/Src/tim.c"
  "${SOLUTION_ROOT}/../Core/Src/usart.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32h7xx_it.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32h7xx_hal_msp.c"
  "${SOLUTION_ROOT}/../Core/Src/startup_main.cpp"
)
target_include_directories(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_Core_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_Core_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_Core PUBLIC
  Group_Application_User_Core_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/freertos.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/dma.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/i2c.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/tim.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/usart.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Drivers/STM32H7xx_HAL_Driver
add_library(Group_Drivers_STM32H7xx_HAL_Driver OBJECT
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c"
)
target_include_directories(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_STM32H7xx_HAL_Driver_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_STM32H7xx_HAL_Driver_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  Group_Drivers_STM32H7xx_HAL_Driver_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Drivers/CMSIS
add_library(Group_Drivers_CMSIS OBJECT
  "${SOLUTION_ROOT}/../Core/Src/system_stm32h7xx.c"
)
target_include_directories(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_CMSIS PUBLIC
  Group_Drivers_CMSIS_ABSTRACTIONS
)

# group applications
add_library(Group_applications OBJECT
  "${SOLUTION_ROOT}/../applications/Src/fdcan_receive.cpp"
  "${SOLUTION_ROOT}/../applications/Src/pid.cpp"
  "${SOLUTION_ROOT}/../applications/Src/pid_user.cpp"
  "${SOLUTION_ROOT}/../applications/Src/timer_user.cpp"
  "${SOLUTION_ROOT}/../applications/Src/chassic.cpp"
  "${SOLUTION_ROOT}/../applications/Src/Communition.cpp"
  "${SOLUTION_ROOT}/../applications/Src/DR16.cpp"
  "${SOLUTION_ROOT}/../applications/Src/my_task.cpp"
  "${SOLUTION_ROOT}/../applications/Src/MT6701.cpp"
  "${SOLUTION_ROOT}/../applications/Src/shoot_param.cpp"
)
target_include_directories(Group_applications PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_applications PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_applications_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_applications_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_applications PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_applications PUBLIC
  Group_applications_ABSTRACTIONS
)

# group bsp/boards
add_library(Group_bsp_boards OBJECT
  "${SOLUTION_ROOT}/../bsp/boards/Src/bsp_fdcan.cpp"
  "${SOLUTION_ROOT}/../bsp/boards/Src/bsp_delay.cpp"
  "${SOLUTION_ROOT}/../bsp/boards/Src/bsp_usart.cpp"
)
target_include_directories(Group_bsp_boards PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_bsp_boards PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_bsp_boards_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_bsp_boards_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_bsp_boards PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_bsp_boards PUBLIC
  Group_bsp_boards_ABSTRACTIONS
)

# group Application/User/USB_DEVICE/App
add_library(Group_Application_User_USB_DEVICE_App OBJECT
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usb_device.c"
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_desc.c"
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_cdc_if.c"
)
target_include_directories(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_USB_DEVICE_App_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_USB_DEVICE_App_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
  $<$<COMPILE_LANGUAGE:CXX>:
    "SHELL:-xc++"
    "SHELL:-fno-exceptions"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
)
target_link_libraries(Group_Application_User_USB_DEVICE_App PUBLIC
  Group_Application_User_USB_DEVICE_App_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/App/usb_device.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_desc.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_cdc_if.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Application/User/USB_DEVICE/Target
add_library(Group_Application_User_USB_DEVICE_Target OBJECT
  "${SOLUTION_ROOT}/../USB_DEVICE/Target/usbd_conf.c"
)
target_include_directories(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
  $<$<COMPILE_LANGUAGE:CXX>:
    "SHELL:-xc++"
    "SHELL:-fno-exceptions"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
)
target_link_libraries(Group_Application_User_USB_DEVICE_Target PUBLIC
  Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/Target/usbd_conf.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Middlewares/FreeRTOS
add_library(Group_Middlewares_FreeRTOS OBJECT
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/list.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/queue.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/timers.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c"
)
target_include_directories(Group_Middlewares_FreeRTOS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Middlewares_FreeRTOS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Middlewares_FreeRTOS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Middlewares_FreeRTOS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Middlewares_FreeRTOS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
  $<$<COMPILE_LANGUAGE:CXX>:
    "SHELL:-xc++"
    "SHELL:-fno-exceptions"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
)
target_link_libraries(Group_Middlewares_FreeRTOS PUBLIC
  Group_Middlewares_FreeRTOS_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/croutine.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/list.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/queue.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/tasks.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/timers.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Middlewares/USB_Device_Library
add_library(Group_Middlewares_USB_Device_Library OBJECT
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
)
target_include_directories(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Middlewares_USB_Device_Library_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Middlewares_USB_Device_Library_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
  $<$<COMPILE_LANGUAGE:CXX>:
    "SHELL:-xc++"
    "SHELL:-fno-exceptions"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-covered-switch-default"
    "SHELL:-Wno-unreachable-code-break"
  >
)
target_link_libraries(Group_Middlewares_USB_Device_Library PUBLIC
  Group_Middlewares_USB_Device_Library_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c" PROPERTIES
  COMPILE_OPTIONS ""
)
