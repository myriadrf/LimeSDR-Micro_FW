
add_library(la9310_cm4)

target_include_directories(la9310_cm4 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/cm4
)

add_subdirectory(drivers)

