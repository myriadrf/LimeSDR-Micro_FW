# Firmware metadata fields (include/firmware_metadata.h), placed right after the ROM boot header. Each image gets its
# own values, and every field is optional: only the given ones are inserted, in this order.
#
#   target_firmware_metadata(<target>
#       [PRODUCT_ID <uint16>]                    # eLMS_DEV board ID
#       [VERSION <major.minor.patch[.revision]>] # each 0..255, revision defaults to 0
#       [GIT_REVISION]                           # read at every build
#       [DESCRIPTION <text>])                    # at most 63 printable ASCII characters
#
# The metadata magic and the END field are always present. The linker script of <target> has to
# KEEP(*(.firmware_metadata)) right after the boot header. Does nothing when FIRMWARE_METADATA is OFF.

option(FIRMWARE_METADATA "Insert metadata fields (product ID, version, git revision, description) after the boot header" ON)

set(FIRMWARE_METADATA_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/firmware_metadata.c.in")
set(FIRMWARE_METADATA_GENERATE_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/FirmwareMetadataGenerate.cmake")

function(target_firmware_metadata target)
    if(NOT FIRMWARE_METADATA)
        return()
    endif()
    cmake_parse_arguments(PARSE_ARGV 1 ARG "GIT_REVISION" "PRODUCT_ID;VERSION;DESCRIPTION" "")
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "target_firmware_metadata(${target}): unknown arguments ${ARG_UNPARSED_ARGUMENTS}")
    endif()
    set(summary "")

    set(has_product_id 0)
    set(product_id 0)
    if(DEFINED ARG_PRODUCT_ID)
        if(NOT ARG_PRODUCT_ID MATCHES "^[0-9]+$" OR ARG_PRODUCT_ID GREATER 65535)
            message(FATAL_ERROR "target_firmware_metadata(${target}): PRODUCT_ID ${ARG_PRODUCT_ID} is not a uint16")
        endif()
        set(has_product_id 1)
        set(product_id ${ARG_PRODUCT_ID})
        list(APPEND summary "product ${product_id}")
    endif()

    set(has_version 0)
    set(major 0)
    set(minor 0)
    set(patch 0)
    set(revision 0)
    if(DEFINED ARG_VERSION)
        if(NOT ARG_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)(\\.([0-9]+))?$")
            message(FATAL_ERROR "target_firmware_metadata(${target}): VERSION ${ARG_VERSION} is not major.minor.patch[.revision]")
        endif()
        set(major ${CMAKE_MATCH_1})
        set(minor ${CMAKE_MATCH_2})
        set(patch ${CMAKE_MATCH_3})
        if(CMAKE_MATCH_5)
            set(revision ${CMAKE_MATCH_5})
        endif()
        foreach(part ${major} ${minor} ${patch} ${revision})
            if(part GREATER 255)
                message(FATAL_ERROR "target_firmware_metadata(${target}): VERSION ${ARG_VERSION} has a part above 255")
            endif()
        endforeach()
        set(has_version 1)
        list(APPEND summary "version ${major}.${minor}.${patch}.${revision}")
    endif()

    set(has_git_revision 0)
    if(ARG_GIT_REVISION)
        set(has_git_revision 1)
        list(APPEND summary "git revision")
    endif()

    set(has_description 0)
    set(description "")
    if(DEFINED ARG_DESCRIPTION)
        string(LENGTH "${ARG_DESCRIPTION}" description_length)
        if(description_length GREATER 63 OR NOT ARG_DESCRIPTION MATCHES "^[ -~]*$" OR ARG_DESCRIPTION MATCHES "[\"\\\\]")
            message(FATAL_ERROR "target_firmware_metadata(${target}): DESCRIPTION must be at most 63 printable ASCII "
                                "characters without quotes or backslashes")
        endif()
        set(has_description 1)
        set(description "${ARG_DESCRIPTION}")
        list(APPEND summary "\"${description}\"")
    endif()

    if(NOT summary)
        set(summary "no fields")
    endif()
    list(JOIN summary ", " summary)

    set(output "${CMAKE_CURRENT_BINARY_DIR}/${target}_firmware_metadata.c")
    # runs at every build, so the git revision follows new commits without reconfiguring
    add_custom_target(${target}-firmware-metadata
        COMMAND ${CMAKE_COMMAND}
            -DTEMPLATE_FILE=${FIRMWARE_METADATA_TEMPLATE}
            -DOUTPUT_FILE=${output}
            -DGIT_DIR=${PROJECT_SOURCE_DIR}
            -DFWMETA_TARGET=${target}
            -DFWMETA_HAS_PRODUCT_ID=${has_product_id}
            -DFWMETA_HAS_VERSION=${has_version}
            -DFWMETA_HAS_GIT_REVISION=${has_git_revision}
            -DFWMETA_HAS_DESCRIPTION=${has_description}
            -DFWMETA_PRODUCT_ID=${product_id}
            -DFWMETA_VERSION_MAJOR=${major}
            -DFWMETA_VERSION_MINOR=${minor}
            -DFWMETA_VERSION_PATCH=${patch}
            -DFWMETA_VERSION_REVISION=${revision}
            -DFWMETA_DESCRIPTION=${description}
            -P ${FIRMWARE_METADATA_GENERATE_SCRIPT}
        BYPRODUCTS ${output}
        DEPENDS ${FIRMWARE_METADATA_TEMPLATE} ${FIRMWARE_METADATA_GENERATE_SCRIPT}
        COMMENT "Generating firmware metadata for ${target}"
        VERBATIM
    )
    add_dependencies(${target} ${target}-firmware-metadata)
    target_sources(${target} PRIVATE ${output})
    target_include_directories(${target} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    message(STATUS "Firmware metadata ${target}: ${summary}")
endfunction()
