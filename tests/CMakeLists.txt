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
