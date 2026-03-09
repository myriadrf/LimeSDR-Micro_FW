
add_library(la9310_cm4 INTERFACE)

target_include_directories(la9310_cm4 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/cm4
)
