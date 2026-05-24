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
