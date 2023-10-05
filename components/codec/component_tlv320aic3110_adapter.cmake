include_guard()
message("component_tlv320aic3110_adapter component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/port/tlv320aic3110/fsl_codec_tlv320aic3110_adapter.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/port/tlv320aic3110
    ${CMAKE_CURRENT_LIST_DIR}/port
)


include(driver_tlv320aic3110)

include(driver_codec)

