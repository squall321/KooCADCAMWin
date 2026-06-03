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
    # slice 8 wave 5 — Aerospace ops (5, metadata-only)
    aerospace_rivet.cpp
    friction_stir_weld_aero.cpp
    rotor_balancing.cpp
    leak_test.cpp
    vibration_qualify.cpp
    # slice 8 wave 5 — Battery + energy (5, metadata-only)
    electrode_coat.cpp
    electrolyte_fill.cpp
    cell_formation.cpp
    separator_lamination.cpp
    cell_seal.cpp
    # slice 8 wave 5 — Recycling + disassembly (5, metadata-only)
    disassemble_step.cpp
    # slice 8 wave 5 — Medical + bioprint (5, metadata-only)
    bioink_print.cpp
    cell_seed.cpp
    sterilize.cpp
    package_medical.cpp
    lyophilize.cpp
    # slice 8 wave 6 — Forging (5)
    open_die_forge.cpp
    closed_die_forge.cpp
    drop_forge.cpp
    ring_roll.cpp
    upsetting.cpp
    # slice 8 wave 6 — Glass + ceramics (5, metadata-only)
    glass_melt.cpp
    glass_blow_mold.cpp
    float_glass.cpp
    temper_glass.cpp
    laminate_glass.cpp
    # slice 8 wave 6 — Cutting tool mfg (5, metadata-only)
    tool_grind.cpp
    tool_recoat.cpp
    edge_prep_hone.cpp
    tool_balancing.cpp
    tool_measurement.cpp
    # slice 8 wave 7 — Food production (5, metadata-only)
    # slice 8 wave 7 — Textile (5, metadata-only)
    weaving.cpp
    knitting.cpp
    fabric_dyeing.cpp
    fabric_finishing.cpp
    fabric_cutting.cpp
    # slice 8 wave 7 — Woodworking (5)
    sawmill_cut.cpp
    wood_planing.cpp
    wood_drilling.cpp
    wood_joinery.cpp
    wood_finishing.cpp
    # slice 8 wave 8 — Packaging (5, metadata-only)
    bag_seal.cpp
    vacuum_pack.cpp
    blister_pack.cpp
    shrink_wrap.cpp
    carton_form.cpp
    # slice 8 wave 8 — Maintenance + repair (5, metadata-only)
    tear_down_inspect.cpp
    wear_inspect.cpp
    replace_part.cpp
    lubrication.cpp
    reassemble.cpp
    # slice 8 wave 8 — Construction (5, metadata-only)
    rebar_bend.cpp
    # slice 8 wave 9 — Process control (5, metadata-only)
    pid_tune.cpp
    recipe_step.cpp
    batch_record_create.cpp
    alarm_threshold_set.cpp
    schedule_parameter_ramp.cpp
    # slice 8 wave 9 — Material handling (5, metadata-only)
    cnc_load_unload.cpp
    automated_storage.cpp
    agv_transport.cpp
    crane_lift.cpp
    hoist_op.cpp
    # slice 8 wave 9 — Logistics + supply (5, metadata-only)
    # slice 8 wave 10 — Pharmaceutical (5, metadata-only)
    tablet_press.cpp
    coating_pan.cpp
    encapsulate.cpp
    lyophilize_pharma.cpp
    sterile_fill.cpp
    # slice 8 wave 10 — Heat exchanger fab (5, metadata-only)
    tube_to_tubesheet_weld.cpp
    expand_tube.cpp
    shell_roll.cpp
    hemispherical_head_form.cpp
    psv_test.cpp
    # slice 8 wave 10 — Optical fiber + photonics (5, metadata-only)
    preform_deposit.cpp
    fiber_draw.cpp
    fiber_splice.cpp
    connector_polish.cpp
    waveguide_pattern.cpp
    # slice 8 wave 11 — Construction-mech (5, metadata-only)
    pipe_install.cpp
    hvac_install.cpp
    electrical_wire.cpp
    fire_protection_install.cpp
    insulation_install.cpp
    # slice 8 wave 11 — Marine (5, metadata-only)
    hull_lay.cpp
    deck_assembly.cpp
    ballast_install.cpp
    propulsion_install.cpp
    paint_marine.cpp
    # slice 8 wave 11 — Energy gen (5, metadata-only)
    turbine_balance.cpp
    generator_winding.cpp
    transformer_wind.cpp
    solar_panel_assemble.cpp
    wind_blade_layup.cpp
    # slice 8 wave 11 — Mining (5, metadata-only)
    # slice 8 wave 11 — Ammunition (5, metadata-only)
    # slice 8 wave 11 — Sports + music (5, metadata-only)
    # slice 8 wave 12 — Agriculture (5, metadata-only)
    # slice 8 wave 12 — Telecom (5, metadata-only)
    # slice 8 wave 12 — Printing (5, metadata-only)
    # slice 8 wave 12 — Apparel (5, metadata-only)
    # slice 8 wave 12 — Safety + destructive test (5, metadata-only)
    drop_test.cpp
    compression_test.cpp
    sled_test.cpp
    tear_test.cpp
    peel_test.cpp
    # slice 8 wave 12 — Steel framing (5, metadata-only)
    cold_form_section.cpp
    gusset_install.cpp
    base_plate_install.cpp
    anchor_bolt_install.cpp
    panel_attach.cpp
    # slice 8 wave 13 — Lab chemistry (5, metadata-only)
    # slice 8 wave 13 — Auto assembly (5, metadata-only)
    chassis_join.cpp
    drivetrain_install.cpp
    interior_install.cpp
    paint_auto.cpp
    qc_auto.cpp
    # slice 8 wave 13 — Renewable energy (5, metadata-only)
    solar_cell_etch.cpp
    fuel_cell_stack_assemble.cpp
    hydrogen_purify.cpp
    biomass_pelletize.cpp
    geothermal_drill.cpp
    # slice 8 wave 13 — Quarry + stone (5, metadata-only)
    quarry_extract.cpp
    sand_wash.cpp
    gravel_grade.cpp
    marble_polish.cpp
    stone_cut.cpp
    # slice 8 wave 13 — Maintenance mechanical (5, metadata-only)
    bearing_replace.cpp
    seal_replace.cpp
    oil_change.cpp
    belt_replace.cpp
    filter_change.cpp
    # slice 8 wave 13 — Weld NDT (5, metadata-only)
    xray_weld_inspect.cpp
    ultrasonic_weld_inspect.cpp
    magnetic_particle_weld.cpp
    penetrant_weld.cpp
    visual_weld_inspect.cpp
    # slice 8 wave 14 — Oil + gas (5, metadata-only)
    # slice 8 wave 14 — Shipyard fabrication (5, metadata-only)
    pickling_passivate.cpp
    pwht.cpp
    # slice 8 wave 14 — Emerging recycling (5, metadata-only)
    battery_recycle.cpp
    pcb_recycle.cpp
    plastic_pyrolysis.cpp
    rare_earth_extract.cpp
    circular_remanufacture.cpp
    # slice 8 wave 14 — Gem + jewelry (5, metadata-only)
    gem_cut.cpp
    gem_polish.cpp
    lost_wax_jewelry.cpp
    electroform_jewelry.cpp
    hallmark_strike.cpp
    # slice 8 wave 14 — EV manufacturing (5, metadata-only)
    motor_winding_ev.cpp
    battery_pack_assemble.cpp
    bms_install.cpp
    charging_port_install.cpp
    inverter_test.cpp
    # slice 8 wave 14 — Space hardware (5, metadata-only)
    rocket_engine_test.cpp
    satellite_assemble.cpp
    solar_array_deploy_test.cpp
    thermal_vacuum_test.cpp
    rocket_propellant_load.cpp
    # slice 8 wave 15 — Microelectronics packaging (5, metadata-only)
    die_attach.cpp
    wire_bond.cpp
    encap_mold.cpp
    bga_reball.cpp
    mems_dicing.cpp
    # slice 8 wave 15 — Cryogenic + vacuum (5, metadata-only)
    cryogenic_anneal.cpp
    vacuum_brazing.cpp
    dewar_assemble.cpp
    gas_purify.cpp
    vacuum_leak_seal.cpp
    # slice 8 wave 15 — Industrial robotics integration (5, metadata-only)
    robot_calibrate.cpp
    end_effector_swap.cpp
    payload_qualify.cpp
    cycle_optimize.cpp
    safety_assess_robot.cpp
    # slice 8 wave 15 — Civil heavy (5, metadata-only)
    cable_stay_tension.cpp
    # slice 8 wave 15 — Rail + train (5, metadata-only)
    # slice 8 wave 15 — Polymer extrusion (5, metadata-only)
    film_extrude.cpp
    profile_extrude.cpp
    sheet_extrude.cpp
    # slice 8 wave 16 — fixturing / tooling (5, REAL compound geometry)
    jig_plate_with_drill_bushings.cpp
    locator_pin_set.cpp
    gauge_block_step.cpp
    vise_jaw_with_v_groove.cpp
    pin_and_diamond_locating_set.cpp
    # slice 8 wave 16-22 auto-wired — compound features (145 skills)
    adjuster_screw_pocket_compound.cpp
    anchor_pad_compound.cpp
    auto_boss_under_hole.cpp
    auto_chamfer_all_outer_edges.cpp
    auto_gusset_corner_brace.cpp
    auto_rib_between_two_walls.cpp
    auto_standoff_floating_point.cpp
    ball_screw_nut_pocket.cpp
    ball_valve_seat_compound.cpp
    banana_jack_receptacle.cpp
    banana_socket_compound.cpp
    banjo_fitting_seat.cpp
    base_bracket_compound.cpp
    bezel_groove_assembly.cpp
    blend_morph_two_shapes.cpp
    blind_threaded_insert_seat.cpp
    bolt_hole_metric_spec.cpp
    bolted_flange_compound.cpp
    boolean_morph_chain.cpp
    box_section_with_endplate.cpp
    breather_vent_compound.cpp
    busbar_lap_joint.cpp
    butt_hinge_pocket.cpp
    butterfly_valve_disc_seat.cpp
    cable_grommet_pass_through.cpp
    cam_actuated_slider.cpp
    cam_follower_roller_seat.cpp
    cam_follower_threaded_seat.cpp
    cam_lock_cavity.cpp
    cam_with_profile.cpp
    captive_nut_pocket.cpp
    captive_screw_pocket_spec.cpp
    caseback_o_ring_groove.cpp
    check_valve_seat_with_stop.cpp
    coil_spring_seat_compound.cpp
    concealed_hinge_cup.cpp
    connector_cutout_with_keepout.cpp
    countersunk_bolt_seat.cpp
    crown_stem_cavity_compound.cpp
    curved_lip_around_face.cpp
    deformation_warp.cpp
    detented_position_slider.cpp
    din_rail_clip_slot.cpp
    din_rail_mount_slot.cpp
    dovetail_mount_compound.cpp
    dowel_pin_h6_bore.cpp
    draft_wall_with_radius.cpp
    dust_lip_seal_seat_compound.cpp
    eccentric_bushing_seat.cpp
    eccentric_shaft_collar.cpp
    expansion_joint_bellows_stub.cpp
    face_seal_compound_compression.cpp
    flange_face_with_gasket_groove.cpp
    flywheel_blank_with_balance_drills.cpp
    flywheel_with_balance.cpp
    full_penetration_butt_prep.cpp
    gas_spring_clevis_pocket.cpp
    gas_strut_hinge_compound.cpp
    gasket_face_with_drain_groove.cpp
    gate_valve_seat_compound.cpp
    gear_blank_with_hub_step.cpp
    governor_arm_with_pivot.cpp
    gusset_plate_compound.cpp
    heat_sink_fin_array.cpp
    helical_gear_teeth.cpp
    helicoil_pilot_compound.cpp
    i_beam_compound_section.cpp
    intermittent_fillet_pattern.cpp
    internal_water_jacket.cpp
    iso_h7_bore_spec.cpp
    isolator_grommet_seat.cpp
    jic_flare_port_seat.cpp
    lead_screw_anti_backlash_pocket.cpp
    leaf_spring_anchor.cpp
    leaf_spring_anchor_compound.cpp
    lifting_lug_pad_eye.cpp
    linear_bushing_seat.cpp
    linear_rail_seat_compound.cpp
    linear_slider_track.cpp
    lip_seal_seat.cpp
    locating_g6_bore.cpp
    louvered_vent.cpp
    lug_with_spring_bar_holes.cpp
    magnetic_latch_pocket.cpp
    manifold_cross_drill_compound.cpp
    needle_bearing_seat_press_fit.cpp
    needle_valve_seat.cpp
    o_ring_groove_as568_spec.cpp
    o_ring_groove_face.cpp
    o_ring_groove_radial.cpp
    over_center_toggle_pocket.cpp
    overcenter_latch.cpp
    parametric_rib_array.cpp
    parametric_sweep_morph.cpp
    partial_pen_fillet_prep.cpp
    partition_wall_with_passthrough.cpp
    pcb_card_edge_socket.cpp
    pcb_standoff_array_under_board.cpp
    pcb_standoff_threaded.cpp
    perforated_grille_pattern.cpp
    piano_hinge_strip.cpp
    pinned_clevis_joint.cpp
    pivot_pin_clevis_compound.cpp
    plain_bushing_bore_with_lube_groove.cpp
    plug_weld_hole.cpp
    press_fit_p7_bore_spec.cpp
    pulley_with_keyway_compound.cpp
    radial_bearing_seat_with_snapring.cpp
    ratchet_pawl_set.cpp
    rj45_socket_cutout.cpp
    rubber_grommet_seat.cpp
    sapphire_glass_seat.cpp
    sealed_bearing_seat_with_shield_relief.cpp
    set_screw_anti_rotation_pocket.cpp
    shaft_with_keyway_step.cpp
    shape_morph_interpolate.cpp
    shim_pocket.cpp
    slip_fit_h11_bore_spec.cpp
    slot_weld_slot.cpp
    snap_action_lock_pocket.cpp
    socket_head_bolt_seat.cpp
    spiral_back_up_ring_groove.cpp
    spline_shaft_compound.cpp
    spring_contact_clip.cpp
    spring_loaded_door_latch.cpp
    sprocket_blank_with_bore.cpp
    sprocket_with_chain_teeth.cpp
    spur_gear_with_real_teeth.cpp
    t_slot_table_groove.cpp
    tab_lock_anti_rotation.cpp
    tapped_hole_metric_spec.cpp
    terminal_block_post.cpp
    threaded_npt_port.cpp
    threaded_through_with_chamfers.cpp
    thrust_bearing_seat_compound.cpp
    tilt_post_for_lcd_panel.cpp
    top_face_recess_with_walls.cpp
    torsion_spring_anchor_compound.cpp
    unc_unf_hole_spec.cpp
    usb_c_port_cutout.cpp
    vent_slot_array.cpp
    vibration_isolator_seat.cpp
    wave_spring_groove.cpp
    x_ring_groove.cpp
    x_ring_groove_spec.cpp
    # slice 10 — sketch-driven CAD-native (5)
    extrude_boss_from_sketch.cpp
    extrude_cut_from_sketch.cpp
    revolve_boss.cpp
    revolve_cut.cpp
    projected_sketch_cut.cpp
    # slice 11 — SolidWorks-style body management (5)
    move_face_along_direction.cpp
    body_split_by_plane.cpp
    body_combine_union.cpp
    body_combine_subtract.cpp
    copy_body_with_transform.cpp
    # slice 12 — CAD-native surfaces (5)
    surface_thicken_to_solid.cpp
    surface_knit_to_shell.cpp
    surface_trim_by_plane.cpp
    surface_offset_from_face.cpp
    surface_fill_4_edges.cpp
    # slice 12 — CAD-native patterns (5)
    linear_pattern.cpp
    circular_pattern.cpp
    mirror_pattern.cpp
    fill_pattern.cpp
    sketch_driven_pattern.cpp
    # slice 12 — CAD-native sweep/loft (5)
    sweep_boss_along_curve.cpp
    sweep_cut_along_curve.cpp
    loft_between_profiles.cpp
    loft_with_guide_curves.cpp
    boundary_surface_4edges.cpp
    # slice 12 — CAD-native shell/mirror/scale (5)
    shell_with_face_skip.cpp
    mirror_body_about_plane.cpp
    scale_body_uniform.cpp
    add_draft_to_face.cpp
    rib_along_path.cpp
    # slice 12 — CAD-native weldment (5)
    structural_member_along_path.cpp
    weldment_end_cap.cpp
    weldment_gusset_triangular.cpp
    miter_corner_trim.cpp
    intermittent_weld_bead.cpp
    # slice 12 — CAD-native reference geometry (5)
    reference_plane_offset.cpp
    reference_plane_at_3pts.cpp
    reference_axis_two_pts.cpp
    reference_point_on_face_uv.cpp
    coordinate_system_from_axes.cpp
    # slice 12 — CAD-native advanced fillet/chamfer (5)
    variable_radius_fillet.cpp
    full_round_fillet.cpp
    face_fillet_two_faces.cpp
    asymmetric_chamfer.cpp
    multi_radius_fillet_table.cpp
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
