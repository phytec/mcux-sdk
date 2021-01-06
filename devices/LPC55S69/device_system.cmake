if(NOT DEVICE_SYSTEM_INCLUDED)

    set(DEVICE_SYSTEM_INCLUDED true CACHE BOOL "device_system component is included.")

    target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/system_LPC55S69_cm33_core0.c
    )

    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/.
    )


    include(device_CMSIS)

endif()