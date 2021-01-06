list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_LIST_DIR}/.
    ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../components/lists
    ${CMAKE_CURRENT_LIST_DIR}/../../components/uart
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/adc16
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/cmp
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/common
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/crc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/dac
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/dmamux
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/edma
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/flash
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/flexio
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/gpio
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/intmux
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/llwu
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpi2c
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpit
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpspi
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lptmr
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpuart
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/mmdvsq
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/pmc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/port
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/rcm
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/rtc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/sim
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/smc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/tpm
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/trgmux
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/trng
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/tsi
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/tstmr
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/wdog32
    ${CMAKE_CURRENT_LIST_DIR}/../../middleware/mmcau
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/assert
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/debug_console_lite
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/misc_utilities
    ${CMAKE_CURRENT_LIST_DIR}/drivers
)


# Copy the cmake components into projects
#    include(driver_llwu)
#    include(driver_flexio_spi_edma)
#    include(CMSIS_Include_dsp)
#    include(component_lpuart_adapter)
#    include(driver_port)
#    include(driver_dmamux)
#    include(driver_lpspi)
#    include(driver_lpi2c_edma)
#    include(driver_gpio)
#    include(component_lists)
#    include(driver_dac)
#    include(device_system)
#    include(driver_tpm)
#    include(driver_lpit)
#    include(driver_cmp)
#    include(device_startup)
#    include(driver_adc16)
#    include(driver_clock)
#    include(utility_debug_console_lite)
#    include(driver_flexio_uart)
#    include(driver_edma)
#    include(driver_trng)
#    include(driver_lpuart)
#    include(driver_flexio_spi)
#    include(middleware_mmcau_common_files)
#    include(middleware_mmcau_cm0p)
#    include(driver_flexio)
#    include(driver_rcm)
#    include(CMSIS_Include_core_cm0plus)
#    include(driver_trgmux)
#    include(driver_tsi_v4)
#    include(driver_flexio_i2c_master)
#    include(driver_sim)
#    include(driver_flash)
#    include(driver_mmdvsq)
#    include(driver_lpspi_edma)
#    include(driver_lpuart_edma)
#    include(device_CMSIS)
#    include(driver_flexio_uart_edma)
#    include(driver_tstmr)
#    include(utility_assert_lite_K32L2A41A)
#    include(utilities_misc_utilities)
#    include(driver_lpi2c)
#    include(CMSIS_Include_common)
#    include(driver_common)
#    include(driver_rtc)
#    include(driver_smc)
#    include(driver_crc)
#    include(driver_wdog32)
#    include(driver_lptmr)
#    include(driver_pmc)
#    include(driver_intmux)
