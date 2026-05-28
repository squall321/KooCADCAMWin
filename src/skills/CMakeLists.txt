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
    # slice 5 — 5-axis / EDM / advanced
    swarf_milling.cpp
    plunge_milling.cpp
    wire_edm.cpp
    sinker_edm.cpp
    spot_face.cpp
    face_milling_5ax.cpp
    # slice 6 — Lathe ops
    turn_external.cpp
    turn_internal.cpp
    face_turn.cpp
    groove_turn.cpp
    parting.cpp
    # slice 6 — Sheet metal
    sheet_punch.cpp
    bend.cpp
    flange.cpp
    hem.cpp
    emboss.cpp
    # slice 6 — Separation cutting
    saw_cut.cpp
    waterjet_cut.cpp
    laser_cut.cpp
    plasma_cut.cpp
    oxyfuel_cut.cpp
    # slice 6 — Advanced milling strategies
    trochoidal_mill.cpp
    adaptive_clearing.cpp
    rest_milling.cpp
    plunge_roughing.cpp
    contour_3ax.cpp
    # slice 6 — Drill/bore variants
    gun_drill.cpp
    deep_hole_peck.cpp
    micro_drill.cpp
    line_boring.cpp
    multi_step_bore.cpp
    # slice 6 — Grinding / honing
    surface_grind.cpp
    cylindrical_grind.cpp
    internal_grind.cpp
    honing_op.cpp
    lapping_op.cpp
    # slice 6 — Mold features
    mold_rib.cpp
    mold_boss.cpp
    snap_fit.cpp
    mold_gate.cpp
    parting_surface.cpp
    # slice 6 — Joining + finishing
    spot_weld.cpp
    seam_weld.cpp
    rivet_hole.cpp
    polish_op.cpp
    sandblast_op.cpp
    # slice 6 — Threading variants
    rigid_tap.cpp
    form_tap.cpp
    thread_mill_external.cpp
    thread_grind.cpp
    thread_chase.cpp
    # slice 6 — 5-axis + inspection
    swarf_5ax.cpp
    contour_5ax.cpp
    indexed_5ax_setup.cpp
    probe_point.cpp
    scan_pattern.cpp
    # slice 7 — Casting + additive (5)
    sand_cast.cpp
    investment_cast.cpp
    3d_print.cpp
    weld_buildup.cpp
    fused_dep_model.cpp
    # slice 7 — Forming (5)
    deep_draw.cpp
    rolling.cpp
    stretching.cpp
    ironing.cpp
    hydroforming.cpp
    # slice 7 — Heat treatment (5, metadata-only)
    anneal.cpp
    harden.cpp
    temper.cpp
    normalize.cpp
    quench.cpp
    # slice 7 — Coating + surface (5, metadata-only)
    anodize.cpp
    electroplate.cpp
    paint_op.cpp
    powder_coat.cpp
    pvd_coat.cpp
    # slice 7 — Welding variants (5)
    tig_weld.cpp
    mig_weld.cpp
    laser_weld.cpp
    friction_stir.cpp
    ultrasonic_weld.cpp
    # slice 7 — Marking (5)
    laser_mark.cpp
    dot_peen_mark.cpp
    electrochemical_etch.cpp
    inkjet_mark.cpp
    label_apply.cpp
    # slice 7 — Fastening (5)
    press_fit.cpp
    threaded_insert.cpp
    heli_coil.cpp
    clinch_nut.cpp
    blind_fastener.cpp
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
