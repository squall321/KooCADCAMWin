# @lat: [[engine/cam-3axis-verify]]
#
# CAM library — slice-1 toolpath generator + collision check.
# Implementation details: [[engine/cam-3axis-verify]]
#
# Slice-1 scope:
#   - Toolpath.cpp           : drill_hole + mill_circular_pocket
#   - CollisionCheck.cpp     : bbox-based 3-axis approximation
#
# Future (M6, NOT in slice-1):
#   - tessellate.cpp         : BRepMesh_IncrementalMesh → STL surface
#   - ocl_bridge.cpp         : OpenCAMLib drop-cutter / waterline
#   - interference_check.cpp : BRepExtrema_DistShapeShape full pipeline
#   - post.cpp               : Fanuc / Heidenhain / Siemens dialect adapters
add_library(koo_cam STATIC
    Toolpath.cpp
    CollisionCheck.cpp
    ToolLibrary.cpp
)

target_include_directories(koo_cam PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_cam SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_cam
    PUBLIC
        koo_skills          # FeatureSignature, ToolingMeta, Workpiece
        TKernel TKMath TKBRep
        TKTopAlgo           # BRepExtrema_DistShapeShape, BRepClass3d_SolidClassifier
        TKPrim              # BRepPrimAPI_MakeCylinder (tool sample solid)
        nlohmann_json::nlohmann_json
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_cam)
