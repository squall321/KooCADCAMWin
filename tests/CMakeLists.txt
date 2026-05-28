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
set_tests_properties(smoke PROPERTIES TIMEOUT 30)

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
set_tests_properties(step_io PROPERTIES TIMEOUT 60)

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
    TIMEOUT 60
    ENVIRONMENT "KOO_TESTS_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data"
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
    set_tests_properties(skill_${name} PROPERTIES TIMEOUT 60)
endmacro()

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

# ── Engine primitive unit tests (slice 4 — helical sweep, polyline offset) ─
macro(koo_add_primitive_test name)
    add_executable(prim_${name}_test engine/primitives/${name}_test.cpp)
    target_link_libraries(prim_${name}_test PRIVATE
        GTest::gtest GTest::gtest_main koo_engine)
    target_include_directories(prim_${name}_test SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
    koo_apply_compiler_flags(prim_${name}_test)
    add_test(NAME prim_${name} COMMAND prim_${name}_test)
    set_tests_properties(prim_${name} PROPERTIES TIMEOUT 60)
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
set_tests_properties(process_plan PROPERTIES TIMEOUT 60)

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
set_tests_properties(plan_editor PROPERTIES TIMEOUT 60)

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
set_tests_properties(llm_bridge PROPERTIES TIMEOUT 30)

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
set_tests_properties(cam PROPERTIES TIMEOUT 60)

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
set_tests_properties(recognizer PROPERTIES TIMEOUT 60)

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
)

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
set_tests_properties(integration_watch PROPERTIES TIMEOUT 120)

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
set_tests_properties(integration_watch_re_roundtrip PROPERTIES TIMEOUT 180)

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
set_tests_properties(integration_watch_complete PROPERTIES TIMEOUT 240)
