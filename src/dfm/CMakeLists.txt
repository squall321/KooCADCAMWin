# @lat: [[architecture/overview#src/dfm/]]
# Placeholder library — DFM rule engine implementation is M3 work.
# See [[engine/dfm-rules]] for the 25-rule catalog.
add_library(koo_dfm STATIC
    placeholder.cpp
)
target_include_directories(koo_dfm PUBLIC ${CMAKE_SOURCE_DIR}/src)
koo_apply_compiler_flags(koo_dfm)
