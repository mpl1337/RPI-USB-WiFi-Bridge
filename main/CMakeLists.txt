set(NUKE_UF2_DIR "${CMAKE_CURRENT_LIST_DIR}/assets")
set(NUKE_UF2 "${NUKE_UF2_DIR}/nuke_universal.uf2")
set(NUKE_UF2_URL "https://github.com/raspberrypi/pico-sdk-prebuilts/releases/download/v2.3.0-0/nuke_universal.uf2")
set(NUKE_UF2_SHA256 "f1d80fcc37161df8cbdf16a58c111208f06234976a470c17699270d212e91754")

if(NOT EXISTS "${NUKE_UF2}")
    file(MAKE_DIRECTORY "${NUKE_UF2_DIR}")
    message(STATUS "Downloading pinned Raspberry Pi nuke_universal.uf2 for embedding")
    file(DOWNLOAD
        "${NUKE_UF2_URL}"
        "${NUKE_UF2}"
        EXPECTED_HASH "SHA256=${NUKE_UF2_SHA256}"
        STATUS NUKE_DOWNLOAD_STATUS
        SHOW_PROGRESS
        TLS_VERIFY ON
    )
    list(GET NUKE_DOWNLOAD_STATUS 0 NUKE_DOWNLOAD_CODE)
    list(GET NUKE_DOWNLOAD_STATUS 1 NUKE_DOWNLOAD_MESSAGE)
    if(NOT NUKE_DOWNLOAD_CODE EQUAL 0)
        file(REMOVE "${NUKE_UF2}")
        message(FATAL_ERROR "Could not download embedded Flash Nuke: ${NUKE_DOWNLOAD_MESSAGE}")
    endif()
endif()

file(SHA256 "${NUKE_UF2}" NUKE_UF2_ACTUAL_SHA256)
if(NOT NUKE_UF2_ACTUAL_SHA256 STREQUAL NUKE_UF2_SHA256)
    message(FATAL_ERROR "nuke_universal.uf2 SHA256 mismatch: ${NUKE_UF2_ACTUAL_SHA256}")
endif()

idf_component_register(
    SRCS
        "app_main.c"
        "bridge_buffers.c"
        "bridge_stats.c"
        "rp_mcu.c"
        "wifi_bridge.c"
        "captive_portal.c"
        "tcp_bridge.c"
        "usb_cdc_bridge.c"
        "http_bridge.c"
        "firmware_update.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        lwip
        esp_event
        esp_netif
        nvs_flash
        esp_http_server
        mbedtls
        esp_timer
        esp_partition
        app_update
        esp_app_format
        fatfs
        usb
    EMBED_FILES
        "assets/nuke_universal.uf2"
)
