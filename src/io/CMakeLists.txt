# @lat: [[architecture/overview#src/io/]]
add_library(koo_io STATIC
    StepIO.cpp
)
target_include_directories(koo_io PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_io SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_io
    PUBLIC
        TKernel TKBRep TKTopAlgo
        TKStep TKXSBase TKXCAF
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_io)
