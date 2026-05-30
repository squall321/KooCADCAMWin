# @lat: [[engine/skills#Layer 5 LLM adapter]]
# Layer 5 — typed editor + NL stub over ProcessPlan.

add_library(koo_adapt STATIC
    PlanEditor.cpp
    NaturalLanguageStub.cpp
    LlmBridge.cpp
)

target_include_directories(koo_adapt PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_adapt SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

# koo_adapt depends on:
#   - koo_skills  → for skill::Workpiece, drill_hole::validate,
#                    skill::SkillError, skill::DFMReport.
#   - koo_process → for the real ProcessPlan / StepInvocation types.
#                    If koo_process is not present in a slice-1 standalone
#                    build, EditOp.hpp falls back to _ProcessPlanStub.hpp
#                    via __has_include — but then this PUBLIC dep must be
#                    dropped to keep the link line clean.
#   - nlohmann_json → for the JSON payload type carried in EditOp.
target_link_libraries(koo_adapt
    PUBLIC
        koo_skills
        koo_process
        nlohmann_json::nlohmann_json
)

# WinHTTP — used by LlmBridge.cpp for the real Anthropic Claude API call.
# Ships with Windows; no vcpkg dep.  PRIVATE because no public header
# exposes WinHTTP types — keeps the public surface portable.
if (WIN32)
    target_link_libraries(koo_adapt PRIVATE winhttp)
endif()

koo_apply_compiler_flags(koo_adapt)
