# @lat: [[engine/skills]]
# Skill library — packaged machining operations (synthesis + analysis).

add_library(koo_skills STATIC
    Workpiece.cpp
    Stock.cpp
    # slice 1
    drill_hole.cpp
    # slice 2 — drilling variants, milling, edge, boring (12 commits, 0607398)
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
    # slice 3 — Tier 1 finishing
    spot_drill.cpp
    ream.cpp
    mill_open_pocket.cpp
    profile_milling.cpp
    # slice 3 — compound macros
    drill_through_hole.cpp
    drill_and_tap.cpp
    bore_and_finish.cpp
    pocket_with_corner_relief.cpp
    # slice 3 — specialty pockets
    mill_keyway.cpp
    dovetail_slot.cpp
    T_slot.cpp
    undercut_milling.cpp
    # slice 3 — Tier 2 skeletons (TODOs for helical / glyph primitives)
    tap_thread.cpp
    thread_mill.cpp
    engrave_text.cpp
    engrave_path.cpp
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
