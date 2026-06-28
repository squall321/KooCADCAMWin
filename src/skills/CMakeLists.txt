# @lat: [[engine/skills]]
# Skill library — packaged machining operations (synthesis + analysis).

add_library(koo_skills STATIC
    Workpiece.cpp
    Stock.cpp
    # slice 1
    drill_hole.cpp
    # B1.2 grammar — generative compounds (recognition lives in koo_re)
    bolt_circle_pattern.cpp
    linear_hole_array.cpp
    rectangular_hole_grid.cpp
    coaxial_step_bore.cpp
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
    dome_boss.cpp
    annular_groove.cpp
    box_boss.cpp
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
    # slice 13 — phone PCB/assembly features (5)
    board_to_board_connector_seat.cpp
    zif_connector_seat.cpp
    antenna_keepout_zone.cpp
    camera_island_step.cpp
    speaker_grille_slot_array.cpp
    # slice 13 — hydraulic ports (5)
    sae_j518_port.cpp
    bspp_g_port.cpp
    npt_taper_port.cpp
    jic_37deg_flare_port.cpp
    orfs_face_seal_port.cpp
    # slice 13 — hydraulic ports (5)  — slice 13b extension family
    sae_j518_port_code61.cpp
    bsp_g_thread_port.cpp
    npt_taper_port_b120.cpp
    jic_37_flare_port.cpp
    orfs_port_4bolt.cpp
    # slice 13 — real involute gears (5)
    spur_gear_involute.cpp
    helical_gear.cpp
    bevel_gear.cpp
    internal_gear.cpp
    rack_involute.cpp
    # slice 13 — advanced mold (5)
    shutoff_surface_explicit.cpp
    side_core_pull_slot.cpp
    lifter_mechanism.cpp
    runner_balanced_h.cpp
    subgate_pin_point.cpp
    # slice 13 — watch bezel/case features (5)
    diving_bezel_120clicks.cpp
    tachymeter_bezel_scale.cpp
    gmt_bezel_24hr.cpp
    screw_down_caseback_6_notches.cpp
    crown_guard_compound.cpp
    # slice 13 — watch bezel/case families (5)
    diving_bezel_unidirectional.cpp
    tachymeter_bezel_engraved.cpp
    case_back_screw_down.cpp
    crown_guard_shoulder_compound.cpp
    lug_integrated_curved.cpp
    # slice 13 — phone PCB mount features (5)
    #   Note: board_to_board_connector_seat and antenna_keepout_zone are
    #   already wired above (under "slice 13 — phone PCB/assembly features");
    #   they were EXTENDED with mounting-post / polygon-pocket support so do
    #   not need re-listing. Only the 3 truly-new skills are added here.
    pcb_standoff_boss_metric.cpp
    zif_connector_window_compound.cpp
    speaker_grille_micro_array.cpp
    # slice 14 — retaining rings + washers (5)
    ext_retaining_ring_groove_din471.cpp
    int_retaining_ring_groove_din472.cpp
    e_clip_groove_din6799.cpp
    spring_lock_washer_seat_din127.cpp
    retaining_ring_din5417_spiral.cpp
    # slice 14 — optical/camera mounts (5)
    c_mount_lens_thread.cpp
    optical_axis_alignment_pin_pair.cpp
    iris_aperture_slot_array.cpp
    kinematic_mount_3point.cpp
    beam_dump_aperture_blackbody.cpp
    # slice 14 — keyway families (5)
    keyway_din6885_parallel_shaft.cpp
    keyway_din6885_parallel_hub.cpp
    keyway_din6886_taper_shaft.cpp
    keyway_woodruff_arc.cpp
    keyway_internal_din6885_through.cpp
    # slice 14 — sheet metal advanced (5)
    k_factor_unfold.cpp
    jog_bend.cpp
    gusset_form.cpp
    hem_open_closed.cpp
    lance_form.cpp
    # slice 14 — EV battery module (5)
    busbar_terminal_post_compound.cpp
    cell_clamp_slot_array.cpp
    cooling_plate_channel_serpentine.cpp
    vent_burst_disc_pocket.cpp
    pcb_lock_tab_compound.cpp
    # slice 15 — automotive engine (5)
    valve_seat_insert_compound.cpp
    piston_oil_gallery_compound.cpp
    crankshaft_journal_with_fillet.cpp
    camshaft_lobe_compound.cpp
    exhaust_manifold_port_compound.cpp
    # slice 15 — heat sink + electronics (5)
    heatsink_fin_pin_array_compound.cpp
    heatsink_blade_fin_extruded.cpp
    thermal_pad_recess_with_to220.cpp
    screw_terminal_strip_array.cpp
    lcd_bezel_window_compound.cpp
    # slice 15 — medical/orthopedic implant (5)
    bone_screw_pilot_compound.cpp
    cannulated_screw_lumen.cpp
    locking_compression_plate_hole.cpp
    hex_drive_recess_implant.cpp
    dental_implant_emergence_taper.cpp
    # slice 15 — aerospace fasteners (5)
    rivet_dimple_compound.cpp
    lockbolt_pin_hole_compound.cpp
    hi_lok_install_hole.cpp
    nas_bolt_pattern_circular.cpp
    panel_fastener_dzus_pocket.cpp
    # slice 15 — HVAC/fluid (5)
    radiator_finger_array.cpp
    hose_barb_compound.cpp
    p_trap_threaded_port.cpp
    pipe_clamp_saddle_compound.cpp
    manifold_block_compound.cpp
    # slice 16 — solar/PV (5)
    mc4_connector_seat_pair.cpp
    pv_junction_box_pottable_pocket.cpp
    mounting_clamp_rail_slot.cpp
    busbar_terminal_lug_compound.cpp
    anti_pid_coating_recess.cpp
    # slice 16 — marine/shipbuilding (5)
    porthole_seal_compound.cpp
    bulkhead_penetration_iso6042.cpp
    hull_plate_strake_compound.cpp
    anchor_chain_locker_pipe_compound.cpp
    bilge_drain_threaded_compound.cpp
    # slice 16 — wind turbine (5)
    blade_root_t_bolt_array.cpp
    pitch_bearing_seat_compound.cpp
    yaw_motor_mount_compound.cpp
    nacelle_panel_lock_quarter_turn.cpp
    generator_stator_slot_pattern.cpp
    # slice 16 — full watch case assemblies (5)
    diver_case_full_compound.cpp
    gmt_chronograph_case_compound.cpp
    square_case_octagonal_compound.cpp
    pocket_watch_hunter_case_compound.cpp
    smart_watch_aluminum_band_lug_compound.cpp
    # slice 16 — agricultural/forestry (5)
    chainsaw_bar_chain_groove.cpp
    tractor_pto_spline_compound.cpp
    harvester_blade_arc_compound.cpp
    hydraulic_quick_coupler_iso7241.cpp
    ag_implement_3point_hitch_pin.cpp
    # slice 16 — audio/musical (5)
    guitar_truss_rod_channel.cpp
    brass_valve_port_compound.cpp
    drum_lug_seat_compound.cpp
    mic_capsule_thread_compound.cpp
    speaker_cone_mount_4hole.cpp
    # slice 16b — marine hardware (5)
    boat_cleat_mounting_compound.cpp
    thru_hull_seacock_flange.cpp
    rudder_gudgeon_pintle_bore.cpp
    winch_drum_base_bolt_circle.cpp
    cutless_bearing_housing_seat.cpp
    # slice 16b — wind turbine (5)
    blade_root_tbolt_compound.cpp
    pitch_bearing_bolt_circle.cpp
    yaw_brake_caliper_mount.cpp
    slip_ring_shaft_bore_keyway.cpp
    nacelle_bedplate_mount_pad.cpp
    # slice 16b — solar/PV mounting (5)
    pv_frame_clamp_tslot.cpp
    solar_rail_splice_channel.cpp
    grounding_lug_layin_seat.cpp
    microinverter_mount_slots.cpp
    tracker_slew_pivot_bore.cpp
    # slice 16b — pro audio hardware (5)
    speaker_basket_spider_seat.cpp
    horn_throat_conical_flare.cpp
    rack_ear_19inch_eia.cpp
    xlr_panel_d_cutout.cpp
    cabinet_handle_dish_recess.cpp
    # slice 16b — watch complications (5)
    chronograph_pusher_tube_bore.cpp
    helium_escape_valve_seat.cpp
    date_window_aperture_compound.cpp
    power_reserve_indicator_slot.cpp
    screw_down_crown_tube_compound.cpp
    # slice 17 — railway / rolling stock (5)
    rail_clip_pandrol_seat.cpp
    axle_box_bearing_housing.cpp
    coupler_knuckle_pin_bore.cpp
    brake_disc_hub_bolt_circle.cpp
    pantograph_carbon_strip_groove.cpp
    # slice 17 — robotics / automation (5)
    harmonic_drive_flange_bore.cpp
    linear_rail_carriage_pocket.cpp
    servo_mount_face_nema.cpp
    cable_carrier_anchor_bracket.cpp
    gripper_finger_dovetail_mount.cpp
    # slice 17 — aerospace structures (5)
    wing_rib_lightening_hole.cpp
    stringer_run_out_cutout.cpp
    fastener_countersink_array_nas.cpp
    access_panel_cutout_radiused.cpp
    fuel_tank_boss_threaded.cpp
    # slice 17 — furniture / cabinetry (5)
    euro_hinge_cup_35mm.cpp
    cam_lock_minifix_bore.cpp
    shelf_pin_hole_row_32mm.cpp
    drawer_slide_pilot_array.cpp
    dowel_joint_boring_pair.cpp
    # slice 17 — bicycle / lightweight frame (5)
    bb_shell_thread_bsa.cpp
    headtube_bearing_seat_zs.cpp
    dropout_thru_axle_12mm.cpp
    bottle_cage_boss_m5.cpp
    internal_cable_port_compound.cpp
    # slice 17 — power transmission (5)
    spur_sprocket_roller_chain.cpp
    v_belt_pulley_groove.cpp
    htd_timing_pulley_teeth.cpp
    flange_coupling_bolt_circle.cpp
    shaft_collar_clamp_split.cpp
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
