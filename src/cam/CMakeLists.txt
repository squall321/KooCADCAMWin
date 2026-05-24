# @lat: [[architecture/overview#src/cam/]]
# Placeholder library — CAM 3-axis verify implementation is M6 work.
# See [[engine/cam-3axis-verify]] for the implementation details.
add_library(koo_cam STATIC
    placeholder.cpp
)
target_include_directories(koo_cam PUBLIC ${CMAKE_SOURCE_DIR}/src)
koo_apply_compiler_flags(koo_cam)
