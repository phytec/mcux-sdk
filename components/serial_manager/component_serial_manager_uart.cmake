#Description: Serial Manager uart; user_visible: True
include_guard(GLOBAL)
message("component_serial_manager_uart component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_component_serial_port_uart.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)

#OR Logic component
if(${MCUX_DEVICE} STREQUAL "MIMXRT1052")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT1064")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MK28FA15")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC54114_cm4")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMX8MQ6")
    include(component_iuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMX8MM6")
    include(component_iuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE15Z7")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "K32L2B31A")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MK64F12")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MK66F18")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE16Z4")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMX8QM6_cm4_core0")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMX8QM6_cm4_core1")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKV11Z7")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKV31F51212")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC54628")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MK22F51212")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE06Z4")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMX8QX6")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT1021")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKE02Z4")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC55S16")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT1062")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMX8MN6")
    include(component_iuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC54S018")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "K32L3A60_cm0plus")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "K32L3A60_cm4")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC54S018M")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKM35Z7")
    include(component_uart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC51U68")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MKL27Z644")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC55S69_cm33_core0")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MCIMX7U5")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT1024")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT1011")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "LPC55S28")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "K32L2A41A")
    include(component_lpuart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT685S_cm33")
    include(component_usart_adapter)
endif()
if(${MCUX_DEVICE} STREQUAL "MIMXRT1015")
    include(component_lpuart_adapter)
endif()

