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
    # slice 8 wave 1 — Assembly (5)
    bolted_joint.cpp
    glued_joint.cpp
    riveted_joint.cpp
    snap_assembly.cpp
    press_assembly.cpp
    # slice 8 wave 1 — Surfacing (5)
    nurbs_patch.cpp
    blend_surface.cpp
    offset_surface.cpp
    sweep_surface.cpp
    loft_surface.cpp
    # slice 8 wave 1 — Sheet variants (5)
    notch.cpp
    lance.cpp
    beading.cpp
    coining.cpp
    draw_bead.cpp
    # slice 8 wave 1 — Lathe variants (5)
    form_turn.cpp
    taper_turn.cpp
    knurl.cpp
    contour_turn.cpp
    drill_lathe.cpp
    # slice 8 wave 1 — Special machining (5)
    ecm.cpp
    edm_drill.cpp
    ultrasonic_machine.cpp
    laser_engrave_deep.cpp
    abrasive_flow.cpp
    # slice 8 wave 1 — Inspection (5)
    cmm_scan.cpp
    white_light_scan.cpp
    photogrammetry_scan.cpp
    ct_scan.cpp
    surface_roughness_test.cpp
    # slice 8 wave 1 — Composite + ceramic (5)
    composite_layup.cpp
    composite_cure.cpp
    ceramic_sinter.cpp
    resin_transfer_mold.cpp
    abrasive_water_jet_drill.cpp
    # slice 8 wave 1 — Plastic injection (5)
    runner_system.cpp
    ejector_pin_hole.cpp
    cooling_channel.cpp
    draft_angle_apply.cpp
    undercut_relief.cpp
    # slice 8 wave 1 — NDT inspection (5, metadata-only)
    ultrasonic_test.cpp
    magnetic_particle_test.cpp
    dye_penetrant_test.cpp
    radiographic_test.cpp
    hardness_test.cpp
    # slice 8 wave 1 — Gear + cam features (5)
    gear_tooth_cut.cpp
    rack_tooth_cut.cpp
    cam_lobe.cpp
    bearing_seat.cpp
    keyway_external.cpp
    # slice 8 wave 1 — Multi-material molding (5)
    insert_molding.cpp
    two_shot_molding.cpp
    overmolding.cpp
    co_extrusion.cpp
    multi_material_3d_print.cpp
    # slice 8 wave 1 — Stock prep + setup (5)
    squaring.cpp
    datum_face_establish.cpp
    fixture_setup.cpp
    rough_stock_cut.cpp
    chuck_grip_zone.cpp
    # slice 8 wave 1 — Stamping (5)
    blanking.cpp
    piercing.cpp
    lance_n_form.cpp
    progressive_die_stage.cpp
    deep_emboss.cpp
    # slice 8 wave 1 — Spinning / flow / wire / spring (5)
    metal_spin.cpp
    flow_form.cpp
    peen_form.cpp
    spring_form.cpp
    wire_form.cpp
    # slice 8 wave 2 — Tube / pipe fab (5)
    tube_swage.cpp
    tube_flare.cpp
    tube_bend.cpp
    tube_hydroform.cpp
    tube_end_form.cpp
    # slice 8 wave 2 — Powder metallurgy (5)
    metal_injection_mold.cpp
    hot_isostatic_press.cpp
    sinter_press.cpp
    powder_atomization.cpp
    powder_blend.cpp
    # slice 8 wave 2 — QC + statistical (5, metadata-only)
    first_article_inspect.cpp
    in_process_inspect.cpp
    statistical_process_ctl.cpp
    batch_release.cpp
    calibration_check.cpp
    # slice 8 wave 2 — Robotic + automation (5, metadata-only)
    robot_pick_place.cpp
    robot_path_program.cpp
    conveyor_transport.cpp
    vibratory_sortation.cpp
    palletize.cpp
    # slice 8 wave 2 — Semiconductor + microfab (5, metadata-only)
    wafer_dice.cpp
    photomask_pattern.cpp
    thin_film_deposit.cpp
    etch_silicon.cpp
    ion_implant.cpp
    # slice 8 wave 3 — Surface engineering (5, metadata-only)
    carburize.cpp
    nitride.cpp
    shot_peen_surface.cpp
    burnish.cpp
    anti_wear_treatment.cpp
    # slice 8 wave 3 — Specialty machining (5)
    micro_milling.cpp
    hard_turning.cpp
    ultra_precision_diamond_turn.cpp
    cryogenic_machining.cpp
    high_speed_machining.cpp
    # slice 8 wave 4 — Optics (5, metadata-only)
    lens_grind.cpp
    lens_polish.cpp
    antireflective_coat.cpp
    dichroic_coat.cpp
    mirror_finish.cpp
    # slice 8 wave 4 — Electronics assembly (5, metadata-only)
    smt_place.cpp
    wave_solder.cpp
    conformal_coat.cpp
    ic_bond.cpp
    flex_attach.cpp
    # slice 8 wave 4 — 3D scanning variants (5, metadata-only)
    structured_light_scan.cpp
    lidar_scan.cpp
    contact_probe_array_scan.cpp
    optical_3d_scan.cpp
    force_feedback_scan.cpp
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
