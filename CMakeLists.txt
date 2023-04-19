if (NOT TARGET pico_renc)
    pico_add_library(pico_renc)

    target_include_directories(pico_renc INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

    target_sources(pico_renc INTERFACE ${CMAKE_CURRENT_LIST_DIR}/renc.c)

    target_link_libraries(pico_renc INTERFACE
                          pico_stdlib
                          pico_util)
endif()
