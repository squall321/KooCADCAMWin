# @lat: [[architecture/overview#src/engine/]]
add_library(koo_engine STATIC
    PlaceholderCylinder.cpp
    WatchFrontModel.cpp
    PhoneFrontModel.cpp
    probe/GeometryProbe.cpp
)
target_include_directories(koo_engine PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_engine SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_engine
    PUBLIC
        TKernel TKMath TKBRep TKTopAlgo TKGeomAlgo
        TKPrim       # BRepPrimAPI_MakeCylinder/MakeBox
        TKFillet     # BRepFilletAPI_MakeFillet (+ ChFi3d_*)
        TKBO         # BRepAlgoAPI_Cut/Fuse (Boolean ops)
        TKOffset     # BRepOffsetAPI_MakePipeShell (helical thread sweep)
        TKShHealing  # ShapeFix; BRepCheck_Analyzer ties pull these in
        nlohmann_json::nlohmann_json
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_engine)
