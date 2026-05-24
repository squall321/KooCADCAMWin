# @lat: [[architecture/overview#src/re/]]
# Placeholder library — Reverse engineering route implementation is M4 work.
# See [[engine/reverse-route]] for the implementation details.
add_library(koo_re STATIC
    placeholder.cpp
)
target_include_directories(koo_re PUBLIC ${CMAKE_SOURCE_DIR}/src)
koo_apply_compiler_flags(koo_re)
