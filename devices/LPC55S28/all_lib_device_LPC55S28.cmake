list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_LIST_DIR}/.
    ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../components/codec
    ${CMAKE_CURRENT_LIST_DIR}/../../components/codec/i2c
    ${CMAKE_CURRENT_LIST_DIR}/../../components/codec/port/wm8904
    ${CMAKE_CURRENT_LIST_DIR}/../../components/codec/wm8904
    ${CMAKE_CURRENT_LIST_DIR}/../../components/i2c
    ${CMAKE_CURRENT_LIST_DIR}/../../components/lists
    ${CMAKE_CURRENT_LIST_DIR}/../../components/osa
    ${CMAKE_CURRENT_LIST_DIR}/../../components/serial_manager
    ${CMAKE_CURRENT_LIST_DIR}/../../components/uart
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/anactrl
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/casper
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/cmp_1
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/common
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/ctimer
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/flexcomm
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/gint
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/hashcrypt
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/iap1
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/inputmux
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpadc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpc_crc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpc_dma
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpc_gpio
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpc_iocon
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/lpc_rtc
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/mrt
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/ostimer
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/pint
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/prince
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/puf
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/rng_1
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/sctimer
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/sdif
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/sysctl
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/utick
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/wwdt
    ${CMAKE_CURRENT_LIST_DIR}/../../../middleware
    ${CMAKE_CURRENT_LIST_DIR}/../../../middleware/sdmmc
    ${CMAKE_CURRENT_LIST_DIR}/../../../middleware/usb
    ${CMAKE_CURRENT_LIST_DIR}/../../../rtos/freertos/freertos_kernel
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/assert
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/debug_console
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/debug_console_lite
    ${CMAKE_CURRENT_LIST_DIR}/../../utilities/misc_utilities
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    ${CMAKE_CURRENT_LIST_DIR}/utilities
)


# Copy the cmake components into projects
#    include(driver_anactrl)
#    include(driver_power)
#    include(driver_lpc_rtc)
#    include(driver_hashcrypt)
#    include(middleware_freertos-kernel_heap_4)
#    include(driver_pint)
#    include(driver_ctimer)
#    include(driver_utick)
#    include(CMSIS_Include_dsp)
#    include(component_usart_adapter)
#    include(driver_wwdt)
#    include(middleware_sdmmc_osa_freertos)
#    include(component_lists)
#    include(driver_lpc_gpio)
#    include(driver_mrt)
#    include(device_system)
#    include(utility_debug_console)
#    include(driver_flexcomm)
#    include(driver_lpadc)
#    include(device_startup)
#    include(middleware_sdmmc_host_sdif_polling)
#    include(driver_clock)
#    include(middleware_freertos-kernel_cm33_nonsecure_port)
#    include(driver_flexcomm_i2s_dma)
#    include(component_osa)
#    include(driver_lpc_dma)
#    include(component_codec_i2c_LPC55S28)
#    include(driver_lpc_crc)
#    include(utility_assert)
#    include(middleware_sdmmc_common)
#    include(driver_rng_1)
#    include(driver_wm8904)
#    include(driver_sdif)
#    include(driver_cmp_1)
#    include(component_serial_manager_swo)
#    include(driver_flexcomm_i2c_dma)
#    include(component_flexcomm_i2c_adapter)
#    include(middleware_freertos-kernel_extension)
#    include(middleware_sdmmc_host_sdif_interrupt)
#    include(driver_flexcomm_usart_dma)
#    include(driver_iap1)
#    include(middleware_usb_common_header)
#    include(driver_flexcomm_spi)
#    include(driver_puf)
#    include(driver_prince)
#    include(driver_flexcomm_i2s)
#    include(middleware_sdmmc_host_sdif)
#    include(middleware_baremetal)
#    include(driver_sctimer)
#    include(middleware_usb_device_common_header)
#    include(driver_flexcomm_usart)
#    include(device_CMSIS)
#    include(middleware_sdmmc_host_sdif_freertos)
#    include(driver_lpc_iocon)
#    include(driver_flexcomm_i2c_freertos)
#    include(utility_debug_console_lite)
#    include(driver_codec)
#    include(utilities_misc_utilities)
#    include(driver_ostimer)
#    include(middleware_freertos-kernel_LPC55S28)
#    include(CMSIS_Include_common)
#    include(driver_common)
#    include(driver_gint)
#    include(middleware_sdmmc_osa_bm)
#    include(driver_flexcomm_usart_freertos)
#    include(utility_assert_lite)
#    include(driver_inputmux)
#    include(driver_inputmux_connections)
#    include(driver_flexcomm_i2c)
#    include(component_serial_manager)
#    include(middleware_sdmmc_sd)
#    include(driver_sysctl)
#    include(driver_casper)
#    include(component_osa_free_rtos)
#    include(component_serial_manager_uart)
#    include(component_wm8904_adapter)
#    include(CMSIS_Include_core_cm33)
#    include(utility_shell)
#    include(driver_reset)
#    include(driver_flexcomm_spi_dma)
#    include(driver_flexcomm_spi_freertos)
#    include(component_osa_bm)
