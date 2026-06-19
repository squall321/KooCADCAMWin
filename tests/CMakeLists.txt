# @lat: [[process/test-strategy]]

find_package(GTest CONFIG REQUIRED)

# ── Smoke test ────────────────────────────────────────────────
add_executable(smoke_test smoke/smoke_test.cpp)
target_link_libraries(smoke_test PRIVATE
    GTest::gtest GTest::gtest_main
    TKernel TKMath TKBRep
    Qt6::Core
)
target_include_directories(smoke_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(smoke_test)
add_test(NAME smoke COMMAND smoke_test)
set_tests_properties(smoke PROPERTIES TIMEOUT 30 LABELS "smoke")

# ── STEP IO round-trip test ───────────────────────────────────
add_executable(step_io_test step_io/step_io_test.cpp)
target_link_libraries(step_io_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_engine
    koo_io
)
target_include_directories(step_io_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(step_io_test)
add_test(NAME step_io COMMAND step_io_test)
set_tests_properties(step_io PROPERTIES TIMEOUT 60 LABELS "io")

# ── make_sample_step tool ─────────────────────────────────────
# Builds a golden cylinder STEP at build time → tests/data/golden_cylinder.step
add_executable(make_sample_step tools/make_sample_step.cpp)
target_link_libraries(make_sample_step PRIVATE koo_engine koo_io)
target_include_directories(make_sample_step SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(make_sample_step)

set(KOO_GOLDEN_STEP "${CMAKE_CURRENT_SOURCE_DIR}/data/golden_cylinder.step")
add_custom_command(
    OUTPUT  "${KOO_GOLDEN_STEP}"
    COMMAND make_sample_step "${KOO_GOLDEN_STEP}"
    DEPENDS make_sample_step
    COMMENT "Generating golden_cylinder.step via make_sample_step"
    VERBATIM
)
add_custom_target(golden_step ALL DEPENDS "${KOO_GOLDEN_STEP}")

# ── koo_analyze CLI ───────────────────────────────────────────
# Load any STEP file and dump the reverse-engineered machining features
# (skill, confidence, measured dimensions).  Usage: koo_analyze <file.step>
add_executable(koo_analyze tools/koo_analyze.cpp)
target_link_libraries(koo_analyze PRIVATE
    koo_io koo_skills koo_re koo_engine koo_process)
target_include_directories(koo_analyze SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(koo_analyze)

# ── koo_modify CLI ────────────────────────────────────────────
# Recover a hole from a foreign STEP and change its diameter on the ORIGINAL
# geometry (preserves the rest of the part). Usage:
#   koo_modify <in.step> <out.step> <tx> <ty> <tz> <new_dia_mm> [tol]
add_executable(koo_modify tools/koo_modify.cpp)
target_link_libraries(koo_modify PRIVATE
    koo_io koo_skills koo_re koo_edit koo_engine koo_process)
target_include_directories(koo_modify SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(koo_modify)

# ── Watch parametric build test (M1.1) ────────────────────────
add_executable(watch_test watch/watch_test.cpp)
target_link_libraries(watch_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_engine
    koo_io
)
target_include_directories(watch_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(watch_test)
add_test(NAME watch COMMAND watch_test)
set_tests_properties(watch PROPERTIES
    TIMEOUT 60
    ENVIRONMENT "KOO_TESTS_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data"
    LABELS "watch"
)

# ── Watch features test (M1.2) — steps 4-6 ──────────────────
add_executable(watch_features_test watch/watch_features_test.cpp)
target_link_libraries(watch_features_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_engine
    koo_io
)
target_include_directories(watch_features_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(watch_features_test)
add_test(NAME watch_features COMMAND watch_features_test)
set_tests_properties(watch_features PROPERTIES
    TIMEOUT 180
    ENVIRONMENT "KOO_TESTS_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data"
    LABELS "watch"
)

# ── ParameterPanel round-trip preservation test (headless Qt offscreen) ──
add_executable(parameter_panel_test gui/parameter_panel_test.cpp)
target_link_libraries(parameter_panel_test PRIVATE
    GTest::gtest
    koo_gui
    koo_engine
)
target_include_directories(parameter_panel_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(parameter_panel_test)
add_test(NAME parameter_panel COMMAND parameter_panel_test)
set_tests_properties(parameter_panel PROPERTIES
    TIMEOUT 90
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    LABELS "gui"
)

# ── PhonePanel round-trip + editing test (headless Qt offscreen, B6.4) ──
add_executable(phone_panel_test gui/phone_panel_test.cpp)
target_link_libraries(phone_panel_test PRIVATE
    GTest::gtest
    koo_gui
    koo_engine
)
target_include_directories(phone_panel_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(phone_panel_test)
add_test(NAME phone_panel COMMAND phone_panel_test)
set_tests_properties(phone_panel PROPERTIES
    TIMEOUT 90
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    LABELS "gui"
)

# ── Skill round-trip tests (slice 1 + slice 2) ────────────────
# Helper macro to keep entries terse.
macro(koo_add_skill_test name)
    add_executable(skill_${name}_test skills/${name}_test.cpp)
    target_link_libraries(skill_${name}_test PRIVATE
        GTest::gtest GTest::gtest_main koo_skills koo_engine koo_io)
    target_include_directories(skill_${name}_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
    koo_apply_compiler_flags(skill_${name}_test)
    add_test(NAME skill_${name} COMMAND skill_${name}_test)
    set_tests_properties(skill_${name} PROPERTIES TIMEOUT 60 LABELS "skill")
endmacro()

# ── Skill feature-chain contract test (breakthrough plan B2.3) ──────
# Verifies apply() preserves the FULL prior feature history (precise tier).
add_executable(skill_contract_test skills/skill_contract_test.cpp)
target_link_libraries(skill_contract_test PRIVATE
    GTest::gtest GTest::gtest_main koo_skills koo_engine koo_io)
target_include_directories(skill_contract_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(skill_contract_test)
add_test(NAME skill_contract COMMAND skill_contract_test)
set_tests_properties(skill_contract PROPERTIES TIMEOUT 120 LABELS "skill;contract")

# ── Skill-synthesis output gate test (modeling roadmap #1) ──────────
# validateOrHeal + Workpiece ctor null-guard: a null/empty/invalid BRep is
# rejected (or healed) rather than silently accepted as a valid Workpiece.
add_executable(workpiece_validate_test skills/workpiece_validate_test.cpp)
target_link_libraries(workpiece_validate_test PRIVATE
    GTest::gtest GTest::gtest_main koo_skills koo_engine koo_io)
target_include_directories(workpiece_validate_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(workpiece_validate_test)
add_test(NAME workpiece_validate COMMAND workpiece_validate_test)
set_tests_properties(workpiece_validate PROPERTIES TIMEOUT 60 LABELS "skill")

# ── koo_corpus CLI: self-labelled synthetic corpus + recall meter ────
# (breakthrough plan B7.1-B7.2)
# Usage: koo_corpus [samples_per_skill=3] [--gate <recall>] [--scale <f>]
add_executable(koo_corpus tools/koo_corpus.cpp)
target_link_libraries(koo_corpus PRIVATE
    koo_io koo_skills koo_re koo_engine koo_process)
target_include_directories(koo_corpus SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(koo_corpus)

# Gate policy (measured 2026-06-11, 15 skills x 3 samples):
#  - recall_pre (recognizer sees its feature) = 1.000 for 13/15 → 0.99 gate.
#  - post-dedupe recall = 1.000 for 12/15 after the B1.5 specificity rule
#    (strict-superset replacement) + counterbore shoulder claim → 0.99 gate.
#    Geometric aliases (drill↔bore↔circular-pocket, drill↔through) count as
#    correct when dimensions match.
#  - exempt (reported, not gated): ream (conf 0.35) / spot_face (0.65) sit
#    below the 0.7 analysis threshold BY DESIGN; bolt_hole_metric_spec
#    cannot win specificity until its recognize() claims an entry face
#    (conf cap 0.78, no entry_face_id — plan B1.5 follow-up).
add_test(NAME corpus_basic
    COMMAND koo_corpus 3 --gate 0.99 --gate-pre 0.99
            --exempt ream,spot_face,bolt_hole_metric_spec)
set_tests_properties(corpus_basic PROPERTIES TIMEOUT 600 LABELS "corpus;re")

# ── Geometry scale-robustness guard (modeling roadmap #4) ─────────────
# The Workpiece-ctor validate/heal gate throws on a BRepCheck-invalid or
# ShapeFix-unhealable output, so synthesising the precise-tier corpus at extreme
# scales (a generic input-space sweep) without the recall gate must still exit
# cleanly — any skill that produces invalid geometry at a parameter extreme
# makes apply() throw and fails this test.  No --gate: recall is allowed to drop
# at extreme scale (DFM windows are absolute, by design); we only assert that
# the geometry itself stays valid.  Measured robust over 0.1x..20x.
add_test(NAME corpus_scale_small COMMAND koo_corpus 2 --scale 0.1)
add_test(NAME corpus_scale_large COMMAND koo_corpus 2 --scale 20)
set_tests_properties(corpus_scale_small corpus_scale_large
    PROPERTIES TIMEOUT 600 LABELS "corpus;re")

# ── Product-spec JSON Schema conformance test (breakthrough plan B6.1) ──
add_executable(spec_schema_test schema/spec_schema_test.cpp)
target_link_libraries(spec_schema_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_engine
    koo_io
)
target_include_directories(spec_schema_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(spec_schema_test)
add_test(NAME spec_schema COMMAND spec_schema_test)
set_tests_properties(spec_schema PROPERTIES
    TIMEOUT 30
    ENVIRONMENT "KOO_SCHEMA_DIR=${CMAKE_SOURCE_DIR}/data/schemas"
    LABELS "io;schema"
)

# ── Product-identity bridge test (multi-product frontier, first slice) ─
# profileForProduct + runDFMForSpec: a design picks its DFM rule set from a
# product tag / inferred keys, not a hard-coded call site.
add_executable(product_identity_test engine/product_identity_test.cpp)
target_link_libraries(product_identity_test PRIVATE
    GTest::gtest GTest::gtest_main koo_engine)
target_include_directories(product_identity_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(product_identity_test)
add_test(NAME product_identity COMMAND product_identity_test)
set_tests_properties(product_identity PROPERTIES TIMEOUT 120 LABELS "engine;dfm")

# ── Product registry test (unified product-tag-driven build/validate) ─
add_executable(product_registry_test engine/product_registry_test.cpp)
target_link_libraries(product_registry_test PRIVATE
    GTest::gtest GTest::gtest_main koo_engine)
target_include_directories(product_registry_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(product_registry_test)
add_test(NAME product_registry COMMAND product_registry_test)
set_tests_properties(product_registry PROPERTIES TIMEOUT 120 LABELS "engine")

# ── GeometryProbe unit test (breakthrough plan B5.2-B5.3) ───────────
add_executable(geometry_probe_test engine/geometry_probe_test.cpp)
target_link_libraries(geometry_probe_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_engine
)
target_include_directories(geometry_probe_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(geometry_probe_test)
add_test(NAME geometry_probe COMMAND geometry_probe_test)
set_tests_properties(geometry_probe PROPERTIES TIMEOUT 60 LABELS "engine")

# ── Feature-chain contract drift guard (breakthrough plan B2) ───────
# Static source scan: fails when any skill stamps a signature without a
# recognized propagation mechanism.  Complements skill_contract (runtime).
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_FOUND)
    add_test(NAME feature_chain_guard
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_feature_chain.py)
    set_tests_properties(feature_chain_guard PROPERTIES
        TIMEOUT 30 LABELS "skill;contract")
endif()

koo_add_skill_test(drill_hole)
koo_add_skill_test(counterbore)
koo_add_skill_test(countersink)
koo_add_skill_test(mill_circular_pocket)
koo_add_skill_test(mill_rect_pocket)
koo_add_skill_test(mill_slot)
koo_add_skill_test(fillet_edge)
koo_add_skill_test(chamfer_edge)
koo_add_skill_test(face_milling)
koo_add_skill_test(bore_cylindrical)
koo_add_skill_test(bore_with_shelf)
koo_add_skill_test(hollow_cavity)
# slice 3
koo_add_skill_test(spot_drill)
koo_add_skill_test(ream)
koo_add_skill_test(mill_open_pocket)
koo_add_skill_test(profile_milling)
koo_add_skill_test(drill_through_hole)
koo_add_skill_test(drill_and_tap)
koo_add_skill_test(bore_and_finish)
koo_add_skill_test(pocket_with_corner_relief)
koo_add_skill_test(mill_keyway)
koo_add_skill_test(dovetail_slot)
koo_add_skill_test(T_slot)
koo_add_skill_test(undercut_milling)
koo_add_skill_test(tap_thread)
koo_add_skill_test(thread_mill)
koo_add_skill_test(engrave_text)
koo_add_skill_test(engrave_path)
# slice 5 — 5-axis / EDM / advanced (6)
koo_add_skill_test(swarf_milling)
koo_add_skill_test(plunge_milling)
koo_add_skill_test(wire_edm)
koo_add_skill_test(sinker_edm)
koo_add_skill_test(spot_face)
koo_add_skill_test(face_milling_5ax)
# slice 6 — Lathe (5)
koo_add_skill_test(turn_external)
koo_add_skill_test(turn_internal)
koo_add_skill_test(face_turn)
koo_add_skill_test(groove_turn)
koo_add_skill_test(parting)
# slice 6 — Sheet metal (5)
koo_add_skill_test(sheet_punch)
koo_add_skill_test(bend)
koo_add_skill_test(flange)
koo_add_skill_test(hem)
koo_add_skill_test(emboss)
# slice 6 — Separation cutting (5)
koo_add_skill_test(saw_cut)
koo_add_skill_test(waterjet_cut)
koo_add_skill_test(laser_cut)
koo_add_skill_test(plasma_cut)
koo_add_skill_test(oxyfuel_cut)
# slice 6 — Advanced milling (5)
koo_add_skill_test(trochoidal_mill)
koo_add_skill_test(adaptive_clearing)
koo_add_skill_test(rest_milling)
koo_add_skill_test(plunge_roughing)
koo_add_skill_test(contour_3ax)
# slice 6 — Drill/bore variants (5)
koo_add_skill_test(gun_drill)
koo_add_skill_test(deep_hole_peck)
koo_add_skill_test(micro_drill)
koo_add_skill_test(line_boring)
koo_add_skill_test(multi_step_bore)
# slice 6 — Grinding / honing (5)
koo_add_skill_test(surface_grind)
koo_add_skill_test(cylindrical_grind)
koo_add_skill_test(internal_grind)
koo_add_skill_test(honing_op)
koo_add_skill_test(lapping_op)
# slice 6 — Mold features (5)
koo_add_skill_test(mold_rib)
koo_add_skill_test(mold_boss)
koo_add_skill_test(snap_fit)
koo_add_skill_test(mold_gate)
koo_add_skill_test(parting_surface)
# slice 6 — Joining / finishing (5)
koo_add_skill_test(spot_weld)
koo_add_skill_test(seam_weld)
koo_add_skill_test(rivet_hole)
koo_add_skill_test(polish_op)
koo_add_skill_test(sandblast_op)
# slice 6 — Threading variants (5)
koo_add_skill_test(rigid_tap)
koo_add_skill_test(form_tap)
koo_add_skill_test(thread_mill_external)
koo_add_skill_test(thread_grind)
koo_add_skill_test(thread_chase)
# slice 6 — 5-axis + inspection (5)
koo_add_skill_test(swarf_5ax)
koo_add_skill_test(contour_5ax)
koo_add_skill_test(indexed_5ax_setup)
koo_add_skill_test(probe_point)
koo_add_skill_test(scan_pattern)
# slice 7 — Casting + additive (5)
koo_add_skill_test(sand_cast)
koo_add_skill_test(investment_cast)
koo_add_skill_test(3d_print)
koo_add_skill_test(weld_buildup)
koo_add_skill_test(fused_dep_model)
# slice 7 — Forming (5)
koo_add_skill_test(deep_draw)
koo_add_skill_test(rolling)
koo_add_skill_test(stretching)
koo_add_skill_test(ironing)
koo_add_skill_test(hydroforming)
# slice 7 — Heat treatment (5, metadata-only)
koo_add_skill_test(anneal)
koo_add_skill_test(harden)
koo_add_skill_test(temper)
koo_add_skill_test(normalize)
koo_add_skill_test(quench)
# slice 7 — Coating + surface (5, metadata-only)
koo_add_skill_test(anodize)
koo_add_skill_test(electroplate)
koo_add_skill_test(paint_op)
koo_add_skill_test(powder_coat)
koo_add_skill_test(pvd_coat)
# slice 7 — Welding variants (5)
koo_add_skill_test(tig_weld)
koo_add_skill_test(mig_weld)
koo_add_skill_test(laser_weld)
koo_add_skill_test(friction_stir)
koo_add_skill_test(ultrasonic_weld)
# slice 7 — Marking (5)
koo_add_skill_test(laser_mark)
koo_add_skill_test(dot_peen_mark)
koo_add_skill_test(electrochemical_etch)
koo_add_skill_test(inkjet_mark)
koo_add_skill_test(label_apply)
# slice 7 — Fastening (5)
koo_add_skill_test(press_fit)
koo_add_skill_test(threaded_insert)
koo_add_skill_test(heli_coil)
koo_add_skill_test(clinch_nut)
koo_add_skill_test(blind_fastener)
# slice 8 wave 1 — Assembly (5)
koo_add_skill_test(bolted_joint)
koo_add_skill_test(glued_joint)
koo_add_skill_test(riveted_joint)
koo_add_skill_test(snap_assembly)
koo_add_skill_test(press_assembly)
# slice 8 wave 1 — Surfacing (5)
koo_add_skill_test(nurbs_patch)
koo_add_skill_test(blend_surface)
koo_add_skill_test(offset_surface)
koo_add_skill_test(sweep_surface)
koo_add_skill_test(loft_surface)
# slice 8 wave 1 — Sheet variants (5)
koo_add_skill_test(notch)
koo_add_skill_test(lance)
koo_add_skill_test(beading)
koo_add_skill_test(coining)
koo_add_skill_test(draw_bead)
# slice 8 wave 1 — Lathe variants (5)
koo_add_skill_test(form_turn)
koo_add_skill_test(taper_turn)
koo_add_skill_test(knurl)
koo_add_skill_test(contour_turn)
koo_add_skill_test(drill_lathe)
# slice 8 wave 1 — Special machining (5)
koo_add_skill_test(ecm)
koo_add_skill_test(edm_drill)
koo_add_skill_test(ultrasonic_machine)
koo_add_skill_test(laser_engrave_deep)
koo_add_skill_test(abrasive_flow)
# slice 8 wave 1 — Inspection (5)
koo_add_skill_test(cmm_scan)
koo_add_skill_test(white_light_scan)
koo_add_skill_test(photogrammetry_scan)
koo_add_skill_test(ct_scan)
koo_add_skill_test(surface_roughness_test)
# slice 8 wave 1 — Composite + ceramic (5)
koo_add_skill_test(composite_layup)
koo_add_skill_test(composite_cure)
koo_add_skill_test(ceramic_sinter)
koo_add_skill_test(resin_transfer_mold)
koo_add_skill_test(abrasive_water_jet_drill)
# slice 8 wave 1 — Plastic injection (5)
koo_add_skill_test(runner_system)
koo_add_skill_test(ejector_pin_hole)
koo_add_skill_test(cooling_channel)
koo_add_skill_test(draft_angle_apply)
koo_add_skill_test(undercut_relief)
# slice 8 wave 1 — NDT inspection (5)
koo_add_skill_test(ultrasonic_test)
koo_add_skill_test(magnetic_particle_test)
koo_add_skill_test(dye_penetrant_test)
koo_add_skill_test(radiographic_test)
koo_add_skill_test(hardness_test)
# slice 8 wave 1 — Gear + cam features (5)
koo_add_skill_test(gear_tooth_cut)
koo_add_skill_test(rack_tooth_cut)
koo_add_skill_test(cam_lobe)
koo_add_skill_test(bearing_seat)
koo_add_skill_test(keyway_external)
# slice 8 wave 1 — Multi-material molding (5)
koo_add_skill_test(insert_molding)
koo_add_skill_test(two_shot_molding)
koo_add_skill_test(overmolding)
koo_add_skill_test(co_extrusion)
koo_add_skill_test(multi_material_3d_print)
# slice 8 wave 1 — Stock prep + setup (5)
koo_add_skill_test(squaring)
koo_add_skill_test(datum_face_establish)
koo_add_skill_test(fixture_setup)
koo_add_skill_test(rough_stock_cut)
koo_add_skill_test(chuck_grip_zone)
# slice 8 wave 1 — Stamping (5)
koo_add_skill_test(blanking)
koo_add_skill_test(piercing)
koo_add_skill_test(lance_n_form)
koo_add_skill_test(progressive_die_stage)
koo_add_skill_test(deep_emboss)
# slice 8 wave 1 — Spinning / flow / wire / spring (5)
koo_add_skill_test(metal_spin)
koo_add_skill_test(flow_form)
koo_add_skill_test(peen_form)
koo_add_skill_test(spring_form)
koo_add_skill_test(wire_form)
# slice 8 wave 2 — Tube / pipe fab (5)
koo_add_skill_test(tube_swage)
koo_add_skill_test(tube_flare)
koo_add_skill_test(tube_bend)
koo_add_skill_test(tube_hydroform)
koo_add_skill_test(tube_end_form)
# slice 8 wave 2 — Powder metallurgy (5)
koo_add_skill_test(metal_injection_mold)
koo_add_skill_test(hot_isostatic_press)
koo_add_skill_test(sinter_press)
koo_add_skill_test(powder_atomization)
koo_add_skill_test(powder_blend)
# slice 8 wave 2 — QC + statistical (5)
koo_add_skill_test(first_article_inspect)
koo_add_skill_test(in_process_inspect)
koo_add_skill_test(statistical_process_ctl)
koo_add_skill_test(batch_release)
koo_add_skill_test(calibration_check)
# slice 8 wave 2 — Robotic + automation (5)
koo_add_skill_test(robot_pick_place)
koo_add_skill_test(robot_path_program)
koo_add_skill_test(conveyor_transport)
koo_add_skill_test(vibratory_sortation)
koo_add_skill_test(palletize)
# slice 8 wave 2 — Semiconductor + microfab (5)
koo_add_skill_test(wafer_dice)
koo_add_skill_test(photomask_pattern)
koo_add_skill_test(thin_film_deposit)
koo_add_skill_test(etch_silicon)
koo_add_skill_test(ion_implant)
# slice 8 wave 3 — Surface engineering (5)
koo_add_skill_test(carburize)
koo_add_skill_test(nitride)
koo_add_skill_test(shot_peen_surface)
koo_add_skill_test(burnish)
koo_add_skill_test(anti_wear_treatment)
# slice 8 wave 3 — Specialty machining (5)
koo_add_skill_test(micro_milling)
koo_add_skill_test(hard_turning)
koo_add_skill_test(ultra_precision_diamond_turn)
koo_add_skill_test(cryogenic_machining)
koo_add_skill_test(high_speed_machining)
# slice 8 wave 4 — Optics (5)
koo_add_skill_test(lens_grind)
koo_add_skill_test(lens_polish)
koo_add_skill_test(antireflective_coat)
koo_add_skill_test(dichroic_coat)
koo_add_skill_test(mirror_finish)
# slice 8 wave 4 — Electronics assembly (5)
koo_add_skill_test(smt_place)
koo_add_skill_test(wave_solder)
koo_add_skill_test(conformal_coat)
koo_add_skill_test(ic_bond)
koo_add_skill_test(flex_attach)
# slice 8 wave 4 — 3D scanning variants (5)
koo_add_skill_test(structured_light_scan)
koo_add_skill_test(lidar_scan)
koo_add_skill_test(contact_probe_array_scan)
koo_add_skill_test(optical_3d_scan)
koo_add_skill_test(force_feedback_scan)
# slice 8 wave 5 — Aerospace ops (5)
koo_add_skill_test(aerospace_rivet)
koo_add_skill_test(friction_stir_weld_aero)
koo_add_skill_test(rotor_balancing)
koo_add_skill_test(leak_test)
koo_add_skill_test(vibration_qualify)
# slice 8 wave 5 — Battery + energy (5)
koo_add_skill_test(electrode_coat)
koo_add_skill_test(electrolyte_fill)
koo_add_skill_test(cell_formation)
koo_add_skill_test(separator_lamination)
koo_add_skill_test(cell_seal)
# slice 8 wave 5 — Recycling + disassembly (5)
koo_add_skill_test(disassemble_step)
# slice 8 wave 5 — Medical + bioprint (5)
koo_add_skill_test(bioink_print)
koo_add_skill_test(cell_seed)
koo_add_skill_test(sterilize)
koo_add_skill_test(package_medical)
koo_add_skill_test(lyophilize)
# slice 8 wave 6 — Forging (5)
koo_add_skill_test(open_die_forge)
koo_add_skill_test(closed_die_forge)
koo_add_skill_test(drop_forge)
koo_add_skill_test(ring_roll)
koo_add_skill_test(upsetting)
# slice 8 wave 6 — Glass + ceramics (5)
koo_add_skill_test(glass_melt)
koo_add_skill_test(glass_blow_mold)
koo_add_skill_test(float_glass)
koo_add_skill_test(temper_glass)
koo_add_skill_test(laminate_glass)
# slice 8 wave 6 — Cutting tool mfg (5)
koo_add_skill_test(tool_grind)
koo_add_skill_test(tool_recoat)
koo_add_skill_test(edge_prep_hone)
koo_add_skill_test(tool_balancing)
koo_add_skill_test(tool_measurement)
# slice 8 wave 7 — Food production (5)
# slice 8 wave 7 — Textile (5)
koo_add_skill_test(weaving)
koo_add_skill_test(knitting)
koo_add_skill_test(fabric_dyeing)
koo_add_skill_test(fabric_finishing)
koo_add_skill_test(fabric_cutting)
# slice 8 wave 7 — Woodworking (5)
koo_add_skill_test(sawmill_cut)
koo_add_skill_test(wood_planing)
koo_add_skill_test(wood_drilling)
koo_add_skill_test(wood_joinery)
koo_add_skill_test(wood_finishing)
# slice 8 wave 8 — Packaging (5)
koo_add_skill_test(bag_seal)
koo_add_skill_test(vacuum_pack)
koo_add_skill_test(blister_pack)
koo_add_skill_test(shrink_wrap)
koo_add_skill_test(carton_form)
# slice 8 wave 8 — Maintenance + repair (5)
koo_add_skill_test(tear_down_inspect)
koo_add_skill_test(wear_inspect)
koo_add_skill_test(replace_part)
koo_add_skill_test(lubrication)
koo_add_skill_test(reassemble)
# slice 8 wave 8 — Construction (5)
koo_add_skill_test(rebar_bend)
# slice 8 wave 9 — Process control (5)
koo_add_skill_test(pid_tune)
koo_add_skill_test(recipe_step)
koo_add_skill_test(batch_record_create)
koo_add_skill_test(alarm_threshold_set)
koo_add_skill_test(schedule_parameter_ramp)
# slice 8 wave 9 — Material handling (5)
koo_add_skill_test(cnc_load_unload)
koo_add_skill_test(automated_storage)
koo_add_skill_test(agv_transport)
koo_add_skill_test(crane_lift)
koo_add_skill_test(hoist_op)
# slice 8 wave 9 — Logistics + supply (5)
# slice 8 wave 10 — Pharmaceutical (5)
koo_add_skill_test(tablet_press)
koo_add_skill_test(coating_pan)
koo_add_skill_test(encapsulate)
koo_add_skill_test(lyophilize_pharma)
koo_add_skill_test(sterile_fill)
# slice 8 wave 10 — Heat exchanger fab (5)
koo_add_skill_test(tube_to_tubesheet_weld)
koo_add_skill_test(expand_tube)
koo_add_skill_test(shell_roll)
koo_add_skill_test(hemispherical_head_form)
koo_add_skill_test(psv_test)
# slice 8 wave 10 — Optical fiber + photonics (5)
koo_add_skill_test(preform_deposit)
koo_add_skill_test(fiber_draw)
koo_add_skill_test(fiber_splice)
koo_add_skill_test(connector_polish)
koo_add_skill_test(waveguide_pattern)
# slice 8 wave 11 — Construction-mech (5)
koo_add_skill_test(pipe_install)
koo_add_skill_test(hvac_install)
koo_add_skill_test(electrical_wire)
koo_add_skill_test(fire_protection_install)
koo_add_skill_test(insulation_install)
# slice 8 wave 11 — Marine (5)
koo_add_skill_test(hull_lay)
koo_add_skill_test(deck_assembly)
koo_add_skill_test(ballast_install)
koo_add_skill_test(propulsion_install)
koo_add_skill_test(paint_marine)
# slice 8 wave 11 — Energy gen (5)
koo_add_skill_test(turbine_balance)
koo_add_skill_test(generator_winding)
koo_add_skill_test(transformer_wind)
koo_add_skill_test(solar_panel_assemble)
koo_add_skill_test(wind_blade_layup)
# slice 8 wave 11 — Mining (5)
# slice 8 wave 11 — Ammunition (5)
# slice 8 wave 11 — Sports + music (5)
# slice 8 wave 12 — Agriculture (5)
# slice 8 wave 12 — Telecom (5)
# slice 8 wave 12 — Printing (5)
# slice 8 wave 12 — Apparel (5)
# slice 8 wave 12 — Safety + destructive test (5)
koo_add_skill_test(drop_test)
koo_add_skill_test(compression_test)
koo_add_skill_test(sled_test)
koo_add_skill_test(tear_test)
koo_add_skill_test(peel_test)
# slice 8 wave 12 — Steel framing (5)
koo_add_skill_test(cold_form_section)
koo_add_skill_test(gusset_install)
koo_add_skill_test(base_plate_install)
koo_add_skill_test(anchor_bolt_install)
koo_add_skill_test(panel_attach)
# slice 8 wave 13 — Lab chemistry (5)
# slice 8 wave 13 — Auto assembly (5)
koo_add_skill_test(chassis_join)
koo_add_skill_test(drivetrain_install)
koo_add_skill_test(interior_install)
koo_add_skill_test(paint_auto)
koo_add_skill_test(qc_auto)
# slice 8 wave 13 — Renewable energy (5)
koo_add_skill_test(solar_cell_etch)
koo_add_skill_test(fuel_cell_stack_assemble)
koo_add_skill_test(hydrogen_purify)
koo_add_skill_test(biomass_pelletize)
koo_add_skill_test(geothermal_drill)
# slice 8 wave 13 — Quarry + stone (5)
koo_add_skill_test(quarry_extract)
koo_add_skill_test(sand_wash)
koo_add_skill_test(gravel_grade)
koo_add_skill_test(marble_polish)
koo_add_skill_test(stone_cut)
# slice 8 wave 13 — Maintenance mechanical (5)
koo_add_skill_test(bearing_replace)
koo_add_skill_test(seal_replace)
koo_add_skill_test(oil_change)
koo_add_skill_test(belt_replace)
koo_add_skill_test(filter_change)
# slice 8 wave 13 — Weld NDT (5)
koo_add_skill_test(xray_weld_inspect)
koo_add_skill_test(ultrasonic_weld_inspect)
koo_add_skill_test(magnetic_particle_weld)
koo_add_skill_test(penetrant_weld)
koo_add_skill_test(visual_weld_inspect)
# slice 8 wave 14 — Oil + gas (5)
# slice 8 wave 14 — Shipyard fab (5)
koo_add_skill_test(pickling_passivate)
koo_add_skill_test(pwht)
# slice 8 wave 14 — Emerging recycling (5)
koo_add_skill_test(battery_recycle)
koo_add_skill_test(pcb_recycle)
koo_add_skill_test(plastic_pyrolysis)
koo_add_skill_test(rare_earth_extract)
koo_add_skill_test(circular_remanufacture)
# slice 8 wave 14 — Gem + jewelry (5)
koo_add_skill_test(gem_cut)
koo_add_skill_test(gem_polish)
koo_add_skill_test(lost_wax_jewelry)
koo_add_skill_test(electroform_jewelry)
koo_add_skill_test(hallmark_strike)
# slice 8 wave 14 — EV manufacturing (5)
koo_add_skill_test(motor_winding_ev)
koo_add_skill_test(battery_pack_assemble)
koo_add_skill_test(bms_install)
koo_add_skill_test(charging_port_install)
koo_add_skill_test(inverter_test)
# slice 8 wave 14 — Space hardware (5)
koo_add_skill_test(rocket_engine_test)
koo_add_skill_test(satellite_assemble)
koo_add_skill_test(solar_array_deploy_test)
koo_add_skill_test(thermal_vacuum_test)
koo_add_skill_test(rocket_propellant_load)
# slice 8 wave 15 — Microelectronics packaging (5)
koo_add_skill_test(die_attach)
koo_add_skill_test(wire_bond)
koo_add_skill_test(encap_mold)
koo_add_skill_test(bga_reball)
koo_add_skill_test(mems_dicing)
# slice 8 wave 15 — Cryogenic + vacuum (5)
koo_add_skill_test(cryogenic_anneal)
koo_add_skill_test(vacuum_brazing)
koo_add_skill_test(dewar_assemble)
koo_add_skill_test(gas_purify)
koo_add_skill_test(vacuum_leak_seal)
# slice 8 wave 15 — Industrial robotics integration (5)
koo_add_skill_test(robot_calibrate)
koo_add_skill_test(end_effector_swap)
koo_add_skill_test(payload_qualify)
koo_add_skill_test(cycle_optimize)
koo_add_skill_test(safety_assess_robot)
# slice 8 wave 15 — Civil heavy (5)
koo_add_skill_test(cable_stay_tension)
# slice 8 wave 15 — Rail + train (5)
# slice 8 wave 15 — Polymer extrusion (5)
koo_add_skill_test(film_extrude)
koo_add_skill_test(profile_extrude)
koo_add_skill_test(sheet_extrude)

# slice 8 wave 16 — fixturing / tooling (5, REAL compound geometry)
koo_add_skill_test(jig_plate_with_drill_bushings)
koo_add_skill_test(locator_pin_set)
koo_add_skill_test(gauge_block_step)
koo_add_skill_test(vise_jaw_with_v_groove)
koo_add_skill_test(pin_and_diamond_locating_set)

# slice 8 wave 16-22 auto-wired — compound features (144 skills)
koo_add_skill_test(adjuster_screw_pocket_compound)
koo_add_skill_test(anchor_pad_compound)
koo_add_skill_test(auto_boss_under_hole)
koo_add_skill_test(auto_chamfer_all_outer_edges)
koo_add_skill_test(auto_gusset_corner_brace)
koo_add_skill_test(auto_rib_between_two_walls)
koo_add_skill_test(auto_standoff_floating_point)
koo_add_skill_test(ball_screw_nut_pocket)
koo_add_skill_test(ball_valve_seat_compound)
koo_add_skill_test(banana_jack_receptacle)
koo_add_skill_test(banana_socket_compound)
koo_add_skill_test(banjo_fitting_seat)
koo_add_skill_test(base_bracket_compound)
koo_add_skill_test(bezel_groove_assembly)
koo_add_skill_test(blend_morph_two_shapes)
koo_add_skill_test(blind_threaded_insert_seat)
koo_add_skill_test(bolt_hole_metric_spec)
koo_add_skill_test(bolted_flange_compound)
koo_add_skill_test(boolean_morph_chain)
koo_add_skill_test(box_section_with_endplate)
koo_add_skill_test(breather_vent_compound)
koo_add_skill_test(busbar_lap_joint)
koo_add_skill_test(butt_hinge_pocket)
koo_add_skill_test(butterfly_valve_disc_seat)
koo_add_skill_test(cable_grommet_pass_through)
koo_add_skill_test(cam_actuated_slider)
koo_add_skill_test(cam_follower_roller_seat)
koo_add_skill_test(cam_follower_threaded_seat)
koo_add_skill_test(cam_lock_cavity)
koo_add_skill_test(cam_with_profile)
koo_add_skill_test(captive_nut_pocket)
koo_add_skill_test(captive_screw_pocket_spec)
koo_add_skill_test(caseback_o_ring_groove)
koo_add_skill_test(check_valve_seat_with_stop)
koo_add_skill_test(coil_spring_seat_compound)
koo_add_skill_test(concealed_hinge_cup)
koo_add_skill_test(connector_cutout_with_keepout)
koo_add_skill_test(countersunk_bolt_seat)
koo_add_skill_test(crown_stem_cavity_compound)
koo_add_skill_test(curved_lip_around_face)
koo_add_skill_test(deformation_warp)
koo_add_skill_test(detented_position_slider)
koo_add_skill_test(din_rail_clip_slot)
koo_add_skill_test(din_rail_mount_slot)
koo_add_skill_test(dovetail_mount_compound)
koo_add_skill_test(dowel_pin_h6_bore)
koo_add_skill_test(draft_wall_with_radius)
koo_add_skill_test(dust_lip_seal_seat_compound)
koo_add_skill_test(eccentric_bushing_seat)
koo_add_skill_test(eccentric_shaft_collar)
koo_add_skill_test(expansion_joint_bellows_stub)
koo_add_skill_test(face_seal_compound_compression)
koo_add_skill_test(flange_face_with_gasket_groove)
koo_add_skill_test(flywheel_blank_with_balance_drills)
koo_add_skill_test(flywheel_with_balance)
koo_add_skill_test(full_penetration_butt_prep)
koo_add_skill_test(gas_spring_clevis_pocket)
koo_add_skill_test(gas_strut_hinge_compound)
koo_add_skill_test(gasket_face_with_drain_groove)
koo_add_skill_test(gear_blank_with_hub_step)
koo_add_skill_test(governor_arm_with_pivot)
koo_add_skill_test(gusset_plate_compound)
koo_add_skill_test(heat_sink_fin_array)
koo_add_skill_test(helical_gear_teeth)
koo_add_skill_test(helicoil_pilot_compound)
koo_add_skill_test(i_beam_compound_section)
koo_add_skill_test(intermittent_fillet_pattern)
koo_add_skill_test(internal_water_jacket)
koo_add_skill_test(iso_h7_bore_spec)
koo_add_skill_test(isolator_grommet_seat)
koo_add_skill_test(jic_flare_port_seat)
koo_add_skill_test(lead_screw_anti_backlash_pocket)
koo_add_skill_test(leaf_spring_anchor)
koo_add_skill_test(leaf_spring_anchor_compound)
koo_add_skill_test(lifting_lug_pad_eye)
koo_add_skill_test(linear_bushing_seat)
koo_add_skill_test(linear_rail_seat_compound)
koo_add_skill_test(linear_slider_track)
koo_add_skill_test(lip_seal_seat)
koo_add_skill_test(locating_g6_bore)
koo_add_skill_test(louvered_vent)
koo_add_skill_test(lug_with_spring_bar_holes)
koo_add_skill_test(magnetic_latch_pocket)
koo_add_skill_test(manifold_cross_drill_compound)
koo_add_skill_test(needle_bearing_seat_press_fit)
koo_add_skill_test(needle_valve_seat)
koo_add_skill_test(o_ring_groove_as568_spec)
koo_add_skill_test(o_ring_groove_face)
koo_add_skill_test(o_ring_groove_radial)
koo_add_skill_test(over_center_toggle_pocket)
koo_add_skill_test(overcenter_latch)
koo_add_skill_test(parametric_rib_array)
koo_add_skill_test(parametric_sweep_morph)
koo_add_skill_test(partial_pen_fillet_prep)
koo_add_skill_test(partition_wall_with_passthrough)
koo_add_skill_test(pcb_card_edge_socket)
koo_add_skill_test(pcb_standoff_array_under_board)
koo_add_skill_test(pcb_standoff_threaded)
koo_add_skill_test(perforated_grille_pattern)
koo_add_skill_test(piano_hinge_strip)
koo_add_skill_test(pinned_clevis_joint)
koo_add_skill_test(pivot_pin_clevis_compound)
koo_add_skill_test(plain_bushing_bore_with_lube_groove)
koo_add_skill_test(plug_weld_hole)
koo_add_skill_test(press_fit_p7_bore_spec)

koo_add_skill_test(pulley_with_keyway_compound)
koo_add_skill_test(radial_bearing_seat_with_snapring)
koo_add_skill_test(ratchet_pawl_set)
koo_add_skill_test(rj45_socket_cutout)
koo_add_skill_test(rubber_grommet_seat)
koo_add_skill_test(sapphire_glass_seat)
koo_add_skill_test(sealed_bearing_seat_with_shield_relief)
koo_add_skill_test(set_screw_anti_rotation_pocket)
koo_add_skill_test(shaft_with_keyway_step)
koo_add_skill_test(shape_morph_interpolate)
koo_add_skill_test(shim_pocket)
koo_add_skill_test(slip_fit_h11_bore_spec)
koo_add_skill_test(slot_weld_slot)

koo_add_skill_test(snap_action_lock_pocket)
koo_add_skill_test(socket_head_bolt_seat)
koo_add_skill_test(spiral_back_up_ring_groove)
koo_add_skill_test(spline_shaft_compound)

koo_add_skill_test(spring_contact_clip)
koo_add_skill_test(spring_loaded_door_latch)
koo_add_skill_test(sprocket_blank_with_bore)
koo_add_skill_test(sprocket_with_chain_teeth)
koo_add_skill_test(spur_gear_with_real_teeth)
koo_add_skill_test(t_slot_table_groove)
koo_add_skill_test(tab_lock_anti_rotation)
koo_add_skill_test(tapped_hole_metric_spec)
koo_add_skill_test(terminal_block_post)
koo_add_skill_test(threaded_npt_port)
koo_add_skill_test(threaded_through_with_chamfers)
koo_add_skill_test(thrust_bearing_seat_compound)
koo_add_skill_test(tilt_post_for_lcd_panel)
koo_add_skill_test(top_face_recess_with_walls)

koo_add_skill_test(torsion_spring_anchor_compound)
koo_add_skill_test(unc_unf_hole_spec)
koo_add_skill_test(usb_c_port_cutout)
koo_add_skill_test(vent_slot_array)
koo_add_skill_test(vibration_isolator_seat)
koo_add_skill_test(wave_spring_groove)
koo_add_skill_test(x_ring_groove)
koo_add_skill_test(x_ring_groove_spec)

# slice 10 — sketch-driven CAD-native (5)
koo_add_skill_test(extrude_boss_from_sketch)
koo_add_skill_test(extrude_cut_from_sketch)
koo_add_skill_test(revolve_boss)
koo_add_skill_test(revolve_cut)
koo_add_skill_test(projected_sketch_cut)

# slice 11 — SolidWorks-style body management (5)
koo_add_skill_test(move_face_along_direction)
koo_add_skill_test(body_split_by_plane)
koo_add_skill_test(body_combine_union)
koo_add_skill_test(body_combine_subtract)
koo_add_skill_test(copy_body_with_transform)

# slice 12 — CAD-native surfaces (5)
koo_add_skill_test(surface_thicken_to_solid)
koo_add_skill_test(surface_knit_to_shell)
koo_add_skill_test(surface_trim_by_plane)
koo_add_skill_test(surface_offset_from_face)
koo_add_skill_test(surface_fill_4_edges)

# slice 12 — CAD-native patterns (5)
koo_add_skill_test(linear_pattern)
koo_add_skill_test(circular_pattern)
koo_add_skill_test(mirror_pattern)
koo_add_skill_test(fill_pattern)
koo_add_skill_test(sketch_driven_pattern)

# slice 12 — CAD-native sweep/loft (5)
koo_add_skill_test(sweep_boss_along_curve)
koo_add_skill_test(sweep_cut_along_curve)
koo_add_skill_test(loft_between_profiles)
koo_add_skill_test(loft_with_guide_curves)
koo_add_skill_test(boundary_surface_4edges)

# slice 12 — CAD-native shell/mirror/scale (5)
koo_add_skill_test(shell_with_face_skip)
koo_add_skill_test(mirror_body_about_plane)
koo_add_skill_test(scale_body_uniform)
koo_add_skill_test(add_draft_to_face)
koo_add_skill_test(rib_along_path)

# slice 12 — CAD-native weldment (5)
koo_add_skill_test(structural_member_along_path)
koo_add_skill_test(weldment_end_cap)
koo_add_skill_test(weldment_gusset_triangular)
koo_add_skill_test(miter_corner_trim)
koo_add_skill_test(intermittent_weld_bead)

# slice 12 — CAD-native reference geometry (5)
koo_add_skill_test(reference_plane_offset)
koo_add_skill_test(reference_plane_at_3pts)
koo_add_skill_test(reference_axis_two_pts)
koo_add_skill_test(reference_point_on_face_uv)
koo_add_skill_test(coordinate_system_from_axes)

# slice 12 — CAD-native advanced fillet/chamfer (5)
koo_add_skill_test(variable_radius_fillet)
koo_add_skill_test(full_round_fillet)
koo_add_skill_test(face_fillet_two_faces)
koo_add_skill_test(asymmetric_chamfer)
koo_add_skill_test(multi_radius_fillet_table)

# slice 13 — phone PCB/assembly features (5)
koo_add_skill_test(board_to_board_connector_seat)
koo_add_skill_test(zif_connector_seat)
koo_add_skill_test(antenna_keepout_zone)
koo_add_skill_test(camera_island_step)
koo_add_skill_test(speaker_grille_slot_array)

# slice 13 — hydraulic ports (5)
koo_add_skill_test(sae_j518_port)
koo_add_skill_test(bspp_g_port)
koo_add_skill_test(npt_taper_port)
koo_add_skill_test(jic_37deg_flare_port)
koo_add_skill_test(orfs_face_seal_port)

# slice 13 — hydraulic ports (5)  — slice 13b extension family
koo_add_skill_test(sae_j518_port_code61)
koo_add_skill_test(bsp_g_thread_port)
koo_add_skill_test(npt_taper_port_b120)
koo_add_skill_test(jic_37_flare_port)
koo_add_skill_test(orfs_port_4bolt)

# slice 13 — advanced mold (5)
koo_add_skill_test(shutoff_surface_explicit)
koo_add_skill_test(side_core_pull_slot)
koo_add_skill_test(lifter_mechanism)
koo_add_skill_test(runner_balanced_h)
koo_add_skill_test(subgate_pin_point)

# slice 13 — watch bezel/case features (5)
koo_add_skill_test(diving_bezel_120clicks)
koo_add_skill_test(tachymeter_bezel_scale)
koo_add_skill_test(gmt_bezel_24hr)
koo_add_skill_test(screw_down_caseback_6_notches)
koo_add_skill_test(crown_guard_compound)

# slice 13 — watch bezel/case families (5)
koo_add_skill_test(diving_bezel_unidirectional)
koo_add_skill_test(tachymeter_bezel_engraved)
koo_add_skill_test(case_back_screw_down)
koo_add_skill_test(crown_guard_shoulder_compound)
koo_add_skill_test(lug_integrated_curved)

# slice 16 — full watch case assemblies (5) — very high-order compounds
koo_add_skill_test(diver_case_full_compound)
set_tests_properties(skill_diver_case_full_compound PROPERTIES TIMEOUT 120)
koo_add_skill_test(gmt_chronograph_case_compound)
set_tests_properties(skill_gmt_chronograph_case_compound PROPERTIES TIMEOUT 120)
koo_add_skill_test(square_case_octagonal_compound)
set_tests_properties(skill_square_case_octagonal_compound PROPERTIES TIMEOUT 120)
koo_add_skill_test(pocket_watch_hunter_case_compound)
set_tests_properties(skill_pocket_watch_hunter_case_compound PROPERTIES TIMEOUT 120)
koo_add_skill_test(smart_watch_aluminum_band_lug_compound)
set_tests_properties(skill_smart_watch_aluminum_band_lug_compound PROPERTIES TIMEOUT 120)

# slice 13 — real involute gears (5)
koo_add_skill_test(spur_gear_involute)
koo_add_skill_test(helical_gear)
set_tests_properties(skill_helical_gear PROPERTIES TIMEOUT 600)
koo_add_skill_test(bevel_gear)
koo_add_skill_test(internal_gear)
koo_add_skill_test(rack_involute)

# slice 13 — phone PCB mount features (5)
koo_add_skill_test(pcb_standoff_boss_metric)
koo_add_skill_test(board_to_board_connector_seat_posts)
koo_add_skill_test(zif_connector_window_compound)
koo_add_skill_test(antenna_keepout_zone_polygon)
koo_add_skill_test(speaker_grille_micro_array)

# slice 14 — optical/camera mounts (5)
koo_add_skill_test(c_mount_lens_thread)
koo_add_skill_test(optical_axis_alignment_pin_pair)
koo_add_skill_test(iris_aperture_slot_array)
koo_add_skill_test(kinematic_mount_3point)
koo_add_skill_test(beam_dump_aperture_blackbody)

# slice 14 — keyway families (5)
koo_add_skill_test(keyway_din6885_parallel_shaft)
koo_add_skill_test(keyway_din6885_parallel_hub)
koo_add_skill_test(keyway_din6886_taper_shaft)
koo_add_skill_test(keyway_woodruff_arc)
koo_add_skill_test(keyway_internal_din6885_through)

# slice 14 — sheet metal advanced (5)
koo_add_skill_test(k_factor_unfold)
koo_add_skill_test(jog_bend)
koo_add_skill_test(gusset_form)
koo_add_skill_test(hem_open_closed)
koo_add_skill_test(lance_form)

# slice 14 — retaining rings + washers (5)
koo_add_skill_test(ext_retaining_ring_groove_din471)
koo_add_skill_test(int_retaining_ring_groove_din472)
koo_add_skill_test(e_clip_groove_din6799)
koo_add_skill_test(spring_lock_washer_seat_din127)
koo_add_skill_test(retaining_ring_din5417_spiral)

# slice 14 — EV battery module (5)
koo_add_skill_test(busbar_terminal_post_compound)
koo_add_skill_test(cell_clamp_slot_array)
koo_add_skill_test(cooling_plate_channel_serpentine)
koo_add_skill_test(vent_burst_disc_pocket)
koo_add_skill_test(pcb_lock_tab_compound)

# slice 15 — heat sink + electronics (5)
koo_add_skill_test(heatsink_fin_pin_array_compound)
koo_add_skill_test(heatsink_blade_fin_extruded)
koo_add_skill_test(thermal_pad_recess_with_to220)
koo_add_skill_test(screw_terminal_strip_array)
koo_add_skill_test(lcd_bezel_window_compound)

# slice 15 — automotive engine (5)
koo_add_skill_test(valve_seat_insert_compound)
koo_add_skill_test(piston_oil_gallery_compound)
koo_add_skill_test(crankshaft_journal_with_fillet)
koo_add_skill_test(camshaft_lobe_compound)
koo_add_skill_test(exhaust_manifold_port_compound)

# slice 15 — medical/orthopedic implant (5)
koo_add_skill_test(bone_screw_pilot_compound)
koo_add_skill_test(cannulated_screw_lumen)
koo_add_skill_test(locking_compression_plate_hole)
koo_add_skill_test(hex_drive_recess_implant)
koo_add_skill_test(dental_implant_emergence_taper)

# slice 15 — aerospace fasteners (5)
koo_add_skill_test(rivet_dimple_compound)
koo_add_skill_test(lockbolt_pin_hole_compound)
koo_add_skill_test(hi_lok_install_hole)
koo_add_skill_test(nas_bolt_pattern_circular)
koo_add_skill_test(panel_fastener_dzus_pocket)

# slice 15 — HVAC/fluid (5)
koo_add_skill_test(radiator_finger_array)
koo_add_skill_test(hose_barb_compound)
koo_add_skill_test(p_trap_threaded_port)
koo_add_skill_test(pipe_clamp_saddle_compound)
koo_add_skill_test(manifold_block_compound)

# slice 16 — solar/PV (5)
koo_add_skill_test(mc4_connector_seat_pair)
koo_add_skill_test(pv_junction_box_pottable_pocket)
koo_add_skill_test(mounting_clamp_rail_slot)
koo_add_skill_test(busbar_terminal_lug_compound)
koo_add_skill_test(anti_pid_coating_recess)

# slice 16 — wind turbine (5)
koo_add_skill_test(blade_root_t_bolt_array)
koo_add_skill_test(pitch_bearing_seat_compound)
koo_add_skill_test(yaw_motor_mount_compound)
koo_add_skill_test(nacelle_panel_lock_quarter_turn)
koo_add_skill_test(generator_stator_slot_pattern)

# slice 16 — agricultural/forestry (5)
koo_add_skill_test(chainsaw_bar_chain_groove)
koo_add_skill_test(tractor_pto_spline_compound)
koo_add_skill_test(harvester_blade_arc_compound)
koo_add_skill_test(hydraulic_quick_coupler_iso7241)
koo_add_skill_test(ag_implement_3point_hitch_pin)

# slice 16 — audio/musical (5)
koo_add_skill_test(guitar_truss_rod_channel)
koo_add_skill_test(brass_valve_port_compound)
koo_add_skill_test(drum_lug_seat_compound)
koo_add_skill_test(mic_capsule_thread_compound)
koo_add_skill_test(speaker_cone_mount_4hole)

# slice 16 — marine/shipbuilding (5)
koo_add_skill_test(porthole_seal_compound)
koo_add_skill_test(bulkhead_penetration_iso6042)
koo_add_skill_test(hull_plate_strake_compound)
koo_add_skill_test(anchor_chain_locker_pipe_compound)
koo_add_skill_test(bilge_drain_threaded_compound)

# slice 16b — marine hardware (5)
koo_add_skill_test(boat_cleat_mounting_compound)
koo_add_skill_test(thru_hull_seacock_flange)
koo_add_skill_test(rudder_gudgeon_pintle_bore)
koo_add_skill_test(winch_drum_base_bolt_circle)
koo_add_skill_test(cutless_bearing_housing_seat)

# slice 16b — wind turbine (5)
koo_add_skill_test(blade_root_tbolt_compound)
koo_add_skill_test(pitch_bearing_bolt_circle)
koo_add_skill_test(yaw_brake_caliper_mount)
koo_add_skill_test(slip_ring_shaft_bore_keyway)
koo_add_skill_test(nacelle_bedplate_mount_pad)

# slice 16b — solar/PV mounting (5)
koo_add_skill_test(pv_frame_clamp_tslot)
koo_add_skill_test(solar_rail_splice_channel)
koo_add_skill_test(grounding_lug_layin_seat)
koo_add_skill_test(microinverter_mount_slots)
koo_add_skill_test(tracker_slew_pivot_bore)

# slice 16b — pro audio hardware (5)
koo_add_skill_test(speaker_basket_spider_seat)
koo_add_skill_test(horn_throat_conical_flare)
koo_add_skill_test(rack_ear_19inch_eia)
koo_add_skill_test(xlr_panel_d_cutout)
koo_add_skill_test(cabinet_handle_dish_recess)

# slice 16b — watch complications (5)
koo_add_skill_test(chronograph_pusher_tube_bore)
koo_add_skill_test(helium_escape_valve_seat)
koo_add_skill_test(date_window_aperture_compound)
koo_add_skill_test(power_reserve_indicator_slot)
koo_add_skill_test(screw_down_crown_tube_compound)

# slice 17 — railway / rolling stock (5)
koo_add_skill_test(rail_clip_pandrol_seat)
koo_add_skill_test(axle_box_bearing_housing)
koo_add_skill_test(coupler_knuckle_pin_bore)
koo_add_skill_test(brake_disc_hub_bolt_circle)
koo_add_skill_test(pantograph_carbon_strip_groove)

# slice 17 — robotics / automation (5)
koo_add_skill_test(harmonic_drive_flange_bore)
koo_add_skill_test(linear_rail_carriage_pocket)
koo_add_skill_test(servo_mount_face_nema)
koo_add_skill_test(cable_carrier_anchor_bracket)
koo_add_skill_test(gripper_finger_dovetail_mount)

# slice 17 — aerospace structures (5)
koo_add_skill_test(wing_rib_lightening_hole)
koo_add_skill_test(stringer_run_out_cutout)
koo_add_skill_test(fastener_countersink_array_nas)
koo_add_skill_test(access_panel_cutout_radiused)
koo_add_skill_test(fuel_tank_boss_threaded)

# slice 17 — furniture / cabinetry (5)
koo_add_skill_test(euro_hinge_cup_35mm)
koo_add_skill_test(cam_lock_minifix_bore)
koo_add_skill_test(shelf_pin_hole_row_32mm)
koo_add_skill_test(drawer_slide_pilot_array)
koo_add_skill_test(dowel_joint_boring_pair)

# slice 17 — bicycle / lightweight frame (5)
koo_add_skill_test(bb_shell_thread_bsa)
koo_add_skill_test(headtube_bearing_seat_zs)
koo_add_skill_test(dropout_thru_axle_12mm)
koo_add_skill_test(bottle_cage_boss_m5)
koo_add_skill_test(internal_cable_port_compound)

# slice 17 — power transmission (5)
koo_add_skill_test(spur_sprocket_roller_chain)
koo_add_skill_test(v_belt_pulley_groove)
koo_add_skill_test(htd_timing_pulley_teeth)
koo_add_skill_test(flange_coupling_bolt_circle)
koo_add_skill_test(shaft_collar_clamp_split)

# ── Compound STEP-roundtrip recognition smoke (slice 8) ──────────────────
add_executable(compound_step_rt_check skills/compound_step_rt_check.cpp)
target_link_libraries(compound_step_rt_check PRIVATE
    GTest::gtest GTest::gtest_main koo_skills koo_engine koo_io)
target_include_directories(compound_step_rt_check SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(compound_step_rt_check)
add_test(NAME compound_step_rt_check COMMAND compound_step_rt_check)
set_tests_properties(compound_step_rt_check PROPERTIES TIMEOUT 180 LABELS "integration")

# ── Engine primitive unit tests (slice 4 — helical sweep, polyline offset) ─
macro(koo_add_primitive_test name)
    add_executable(prim_${name}_test engine/primitives/${name}_test.cpp)
    target_link_libraries(prim_${name}_test PRIVATE
        GTest::gtest GTest::gtest_main koo_engine)
    target_include_directories(prim_${name}_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
    koo_apply_compiler_flags(prim_${name}_test)
    add_test(NAME prim_${name} COMMAND prim_${name}_test)
    set_tests_properties(prim_${name} PROPERTIES TIMEOUT 60 LABELS "primitive")
endmacro()

koo_add_primitive_test(helical_sweep)
koo_add_primitive_test(polyline_offset)

# ── Layer 3 ProcessPlan + Executor tests ──────────────────────
add_executable(process_plan_test process/process_plan_test.cpp)
target_link_libraries(process_plan_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_process
    koo_skills
    koo_engine
)
target_include_directories(process_plan_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(process_plan_test)
add_test(NAME process_plan COMMAND process_plan_test)
set_tests_properties(process_plan PROPERTIES TIMEOUT 60 LABELS "process")

# ── B3.4 recovered-parameter clamp ────────────────────────────
add_executable(param_clamp_test process/param_clamp_test.cpp)
target_link_libraries(param_clamp_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_process
    koo_skills
    koo_engine
)
target_include_directories(param_clamp_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(param_clamp_test)
add_test(NAME param_clamp COMMAND param_clamp_test)
set_tests_properties(param_clamp PROPERTIES TIMEOUT 60 LABELS "process")

# ── Layer 5 LLM adapter / PlanEditor tests ────────────────────
add_executable(plan_editor_test adapt/plan_editor_test.cpp)
target_link_libraries(plan_editor_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_adapt
    koo_process
    koo_skills
    koo_engine
)
target_include_directories(plan_editor_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(plan_editor_test)
add_test(NAME plan_editor COMMAND plan_editor_test)
set_tests_properties(plan_editor PROPERTIES TIMEOUT 60 LABELS "process")

# ── Layer 5 LlmBridge tests ───────────────────────────────────
add_executable(llm_bridge_test adapt/llm_bridge_test.cpp)
target_link_libraries(llm_bridge_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_adapt
    koo_process
    koo_skills
    koo_engine
)
target_include_directories(llm_bridge_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(llm_bridge_test)
add_test(NAME llm_bridge COMMAND llm_bridge_test)
set_tests_properties(llm_bridge PROPERTIES TIMEOUT 30 LABELS "process")

# ── Layer 6 CAM toolpath + collision-check tests ──────────────
add_executable(cam_test cam/cam_test.cpp)
target_link_libraries(cam_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_cam
    koo_skills
    koo_engine
)
target_include_directories(cam_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(cam_test)
add_test(NAME cam COMMAND cam_test)
set_tests_properties(cam PROPERTIES TIMEOUT 60 LABELS "cam")

# CAM integration glue (planAndVerify: generate toolpaths + collision-check).
add_executable(toolpath_plan_test cam/toolpath_plan_test.cpp)
target_link_libraries(toolpath_plan_test PRIVATE
    GTest::gtest GTest::gtest_main koo_cam koo_skills koo_engine)
target_include_directories(toolpath_plan_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(toolpath_plan_test)
add_test(NAME toolpath_plan COMMAND toolpath_plan_test)
set_tests_properties(toolpath_plan PROPERTIES TIMEOUT 60 LABELS "cam")

# ── Sketch-extrusion RE round-trip (recognize -> regenerate) ──
add_executable(extrude_roundtrip_test re/extrude_roundtrip_test.cpp)
target_link_libraries(extrude_roundtrip_test PRIVATE
    GTest::gtest GTest::gtest_main koo_re koo_process koo_skills koo_engine)
target_include_directories(extrude_roundtrip_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(extrude_roundtrip_test)
add_test(NAME extrude_roundtrip COMMAND extrude_roundtrip_test)
set_tests_properties(extrude_roundtrip PROPERTIES TIMEOUT 60 LABELS "re")

add_executable(revolve_roundtrip_test re/revolve_roundtrip_test.cpp)
target_link_libraries(revolve_roundtrip_test PRIVATE
    GTest::gtest GTest::gtest_main koo_re koo_process koo_skills koo_engine)
target_include_directories(revolve_roundtrip_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(revolve_roundtrip_test)
add_test(NAME revolve_roundtrip COMMAND revolve_roundtrip_test)
set_tests_properties(revolve_roundtrip PROPERTIES TIMEOUT 60 LABELS "re")

# ── Layer 4 RE / Recognizer tests ─────────────────────────────
add_executable(recognizer_test re/recognizer_test.cpp)
target_link_libraries(recognizer_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re
    koo_process
    koo_skills
    koo_engine
)
target_include_directories(recognizer_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(recognizer_test)
add_test(NAME recognizer COMMAND recognizer_test)
set_tests_properties(recognizer PROPERTIES TIMEOUT 60 LABELS "re")

# ── Un-cap safety scan (breakthrough plan B1.x measurement) ───────
add_executable(fpscan_test re/fpscan_test.cpp)
target_link_libraries(fpscan_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re koo_io koo_skills koo_engine)
target_include_directories(fpscan_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(fpscan_test)
add_test(NAME fpscan COMMAND fpscan_test)
set_tests_properties(fpscan PROPERTIES TIMEOUT 120 LABELS "re;corpus")

# ── B1.2 declarative compound grammar (orthodox un-cap path) ──────
add_executable(compound_grammar_test re/compound_grammar_test.cpp)
target_link_libraries(compound_grammar_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re koo_io koo_skills koo_engine)
target_include_directories(compound_grammar_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(compound_grammar_test)
add_test(NAME compound_grammar COMMAND compound_grammar_test)
set_tests_properties(compound_grammar PROPERTIES TIMEOUT 120 LABELS "re")

# ── B1.2 grammar — generative compound (recognise → edit → regenerate) ──
add_executable(bolt_circle_pattern_test skills/bolt_circle_pattern_test.cpp)
target_link_libraries(bolt_circle_pattern_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re koo_io koo_process koo_skills koo_engine)
target_include_directories(bolt_circle_pattern_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(bolt_circle_pattern_test)
add_test(NAME bolt_circle_pattern COMMAND bolt_circle_pattern_test)
set_tests_properties(bolt_circle_pattern PROPERTIES TIMEOUT 120 LABELS "skills;re")

add_executable(linear_hole_array_test skills/linear_hole_array_test.cpp)
target_link_libraries(linear_hole_array_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re koo_io koo_process koo_skills koo_engine)
target_include_directories(linear_hole_array_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(linear_hole_array_test)
add_test(NAME linear_hole_array COMMAND linear_hole_array_test)
set_tests_properties(linear_hole_array PROPERTIES TIMEOUT 120 LABELS "skills;re")

add_executable(rectangular_hole_grid_test skills/rectangular_hole_grid_test.cpp)
target_link_libraries(rectangular_hole_grid_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re koo_io koo_process koo_skills koo_engine)
target_include_directories(rectangular_hole_grid_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(rectangular_hole_grid_test)
add_test(NAME rectangular_hole_grid COMMAND rectangular_hole_grid_test)
set_tests_properties(rectangular_hole_grid PROPERTIES TIMEOUT 120 LABELS "skills;re")

add_executable(coaxial_step_bore_test skills/coaxial_step_bore_test.cpp)
target_link_libraries(coaxial_step_bore_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_re koo_io koo_process koo_skills koo_engine)
target_include_directories(coaxial_step_bore_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(coaxial_step_bore_test)
add_test(NAME coaxial_step_bore COMMAND coaxial_step_bore_test)
set_tests_properties(coaxial_step_bore PROPERTIES TIMEOUT 120 LABELS "skills;re")

# ── Phone parametric build test (M2-phase-1) ──────────────────
add_executable(phone_test phone/phone_test.cpp)
target_link_libraries(phone_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_engine
    koo_io
)
target_include_directories(phone_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(phone_test)
add_test(NAME phone COMMAND phone_test)
set_tests_properties(phone PROPERTIES
    TIMEOUT 60
    ENVIRONMENT "KOO_TESTS_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data"
    LABELS "phone"
)

# ── Driven-dimension pre-pass test (modeling roadmap #3a) ─────
add_executable(spec_expr_test io/spec_expr_test.cpp)
target_link_libraries(spec_expr_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_io koo_engine)
target_include_directories(spec_expr_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(spec_expr_test)
add_test(NAME spec_expr COMMAND spec_expr_test)
set_tests_properties(spec_expr PROPERTIES TIMEOUT 60 LABELS "io")

# ── JsonSpec validate/roundtrip test (M1.1) ───────────────────
add_executable(json_spec_test json_spec/json_spec_test.cpp)
target_link_libraries(json_spec_test PRIVATE
    GTest::gtest GTest::gtest_main
    koo_io
)
target_include_directories(json_spec_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(json_spec_test)
add_test(NAME json_spec COMMAND json_spec_test)
set_tests_properties(json_spec PROPERTIES
    TIMEOUT 30
    ENVIRONMENT "KOO_TESTS_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data"
    LABELS "io"
)

# ── Integration: watch built as a skill::ProcessPlan ──────────
# Deliverable 1 — synthesise a round watch case using only the 8 skills
# registered in the Executor dispatch table.  Verifies bbox/volume/
# BRepCheck on the resulting workpiece.
add_executable(integration_watch integration/watch_as_skill_sequence_test.cpp)
target_link_libraries(integration_watch PRIVATE
    GTest::gtest GTest::gtest_main
    koo_skills koo_process koo_re koo_io koo_engine)
target_include_directories(integration_watch SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(integration_watch)
add_test(NAME integration_watch COMMAND integration_watch)
set_tests_properties(integration_watch PROPERTIES TIMEOUT 120 LABELS "integration")

# ── Integration: full RE round-trip on watch workpiece ────────
# Deliverable 2 — stock → exec → STEP → exec → analyze → inferPlan →
# exec; verifies the inferred plan's skill set & bbox/volume close the
# bidirectional loop on a watch-style cylindrical workpiece.
add_executable(integration_watch_re_roundtrip
    integration/watch_re_roundtrip_test.cpp)
target_link_libraries(integration_watch_re_roundtrip PRIVATE
    GTest::gtest GTest::gtest_main
    koo_skills koo_process koo_re koo_io koo_adapt koo_engine)
target_include_directories(integration_watch_re_roundtrip SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(integration_watch_re_roundtrip)
add_test(NAME integration_watch_re_roundtrip COMMAND integration_watch_re_roundtrip)
set_tests_properties(integration_watch_re_roundtrip PROPERTIES TIMEOUT 180 LABELS "integration")

# ── Integration: reframe loop — original vision proven with measured dims ──
# synth frame -> STEP(strip metadata) -> geometric recognize -> part moves
# +15mm -> adapt -> re-execute -> STEP -> recognize -> assert holes followed.
add_executable(integration_frame_reframe
    integration/frame_reframe_loop_test.cpp)
target_link_libraries(integration_frame_reframe PRIVATE
    GTest::gtest GTest::gtest_main
    koo_skills koo_process koo_re koo_io koo_parts koo_engine)
target_include_directories(integration_frame_reframe SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(integration_frame_reframe)
add_test(NAME integration_frame_reframe COMMAND integration_frame_reframe)
set_tests_properties(integration_frame_reframe PROPERTIES TIMEOUT 180 LABELS "integration")

# ── Integration: modify a foreign hole in place + re-verify (koo_modify core) ─
add_executable(integration_modify_in_place
    integration/modify_in_place_test.cpp)
target_link_libraries(integration_modify_in_place PRIVATE
    GTest::gtest GTest::gtest_main
    koo_skills koo_process koo_re koo_edit koo_io koo_engine)
target_include_directories(integration_modify_in_place SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(integration_modify_in_place)
add_test(NAME integration_modify_in_place COMMAND integration_modify_in_place)
set_tests_properties(integration_modify_in_place PROPERTIES TIMEOUT 120 LABELS "integration")

# ── Parts layout + datum graph + reframePlanForMoves (Layer 6) ────────────
# (was previously unregistered — never built/run; now wired in.)
add_executable(parts_layout_test parts/parts_layout_test.cpp)
target_link_libraries(parts_layout_test PRIVATE
    GTest::gtest GTest::gtest_main koo_parts koo_process koo_io)
target_include_directories(parts_layout_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(parts_layout_test)
add_test(NAME parts_layout COMMAND parts_layout_test)
set_tests_properties(parts_layout PROPERTIES TIMEOUT 60 LABELS "process")

# ── Integration: COMPLETE watch as a 11-step ProcessPlan ──────
# Slice-1 end-to-end demo — build a full M1.5 watch using ONLY the
# 28 dispatched skills, then round-trip through (a) the JSON plan
# serializer, (b) STEP IO + Recognizer, and (c) WatchFrontModel::runDFM.
add_executable(integration_watch_complete
    integration/watch_complete_test.cpp)
target_link_libraries(integration_watch_complete PRIVATE
    GTest::gtest GTest::gtest_main
    koo_skills koo_process koo_re koo_adapt koo_io koo_engine)
target_include_directories(integration_watch_complete SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(integration_watch_complete)
add_test(NAME integration_watch_complete COMMAND integration_watch_complete)
set_tests_properties(integration_watch_complete PROPERTIES TIMEOUT 240 LABELS "integration")

# ── Integration: bearing-housing compound-skill process plan ──
# Slice-8 end-to-end demo — build a realistic bearing-housing using a 9-step
# Layer-3 ProcessPlan exercising 6 distinct skills (squaring +
# bore_cylindrical + radial_bearing_seat_with_snapring +
# socket_head_bolt_seat x 4 + o_ring_groove_face + chamfer_edge).
# At commit HEAD, 4 of the 6 skills are NOT YET in Executor::dispatchTable()
# (slice-9 dispatcher patch required); the forward-execution tests are
# therefore DISABLED_-prefixed.  TEST 0 (plan JSON round-trip) and TEST 4
# (dispatch-table audit) run unconditionally.
add_executable(integration_bearing_housing
    integration/bearing_housing_test.cpp)
target_link_libraries(integration_bearing_housing PRIVATE
    GTest::gtest GTest::gtest_main
    koo_skills koo_process koo_re koo_io koo_engine)
target_include_directories(integration_bearing_housing SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
koo_apply_compiler_flags(integration_bearing_housing)
add_test(NAME integration_bearing_housing COMMAND integration_bearing_housing)
set_tests_properties(integration_bearing_housing PROPERTIES TIMEOUT 240 LABELS "integration")
