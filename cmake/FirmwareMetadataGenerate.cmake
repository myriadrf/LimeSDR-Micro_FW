# Build time step of target_firmware_metadata() (FirmwareMetadata.cmake): reads the git revision and writes OUTPUT_FILE
# from TEMPLATE_FILE, if FWMETA_HAS_GIT_REVISION. Everything else comes in as FWMETA_* variables.
#   cmake -DTEMPLATE_FILE=... -DOUTPUT_FILE=... -DGIT_DIR=... -DFWMETA_...=... -P FirmwareMetadataGenerate.cmake
set(FWMETA_GIT_REVISION "0")
find_package(Git QUIET)
if(NOT FWMETA_HAS_GIT_REVISION)
    # field not requested
elseif(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${GIT_DIR}" rev-parse --short=8 HEAD
        OUTPUT_VARIABLE git_hash
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE git_result
        ERROR_QUIET)
    if(git_result EQUAL 0 AND git_hash MATCHES "^[0-9a-f]+$")
        string(SUBSTRING "${git_hash}" 0 8 git_hash)
        set(FWMETA_GIT_REVISION "0x${git_hash}")
    endif()
endif()
if(FWMETA_HAS_GIT_REVISION AND FWMETA_GIT_REVISION STREQUAL "0")
    message(WARNING "Firmware metadata (${FWMETA_TARGET}): git revision unknown, using 0")
endif()

configure_file("${TEMPLATE_FILE}" "${OUTPUT_FILE}" @ONLY)
