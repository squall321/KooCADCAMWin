# @lat: [[engine/skills]]
# Skill library — packaged machining operations (synthesis + analysis).

add_library(koo_skills STATIC
    Workpiece.cpp
    Stock.cpp
    drill_hole.cpp
    counterbore.cpp
    countersink.cpp
    mill_circular_pocket.cpp
    mill_rect_pocket.cpp
    mill_slot.cpp
    fillet_edge.cpp
    chamfer_edge.cpp
    face_milling.cpp
    bore_cylindrical.cpp
    bore_with_shelf.cpp
    hollow_cavity.cpp
)

target_include_directories(koo_skills PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_skills SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_skills
    PUBLIC
        koo_engine          # for prim::*
        TKernel TKMath TKBRep TKTopAlgo TKGeomAlgo
        TKPrim              # BRepPrimAPI_MakeCylinder/MakeBox
        TKFillet
        TKBO                # BRepAlgoAPI_Cut/Fuse
        TKShHealing
        nlohmann_json::nlohmann_json
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_skills)
