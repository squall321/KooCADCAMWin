# @lat: [[engine/reverse-route]]
# Layer 4 — Reverse engineering pipeline.
# Recognizer scans a workpiece by invoking every registered skill's
# recognize() and assembles the candidates into a ProcessPlan.

add_library(koo_re STATIC
    Recognizer.cpp
    CompoundGrammar.cpp
)

target_include_directories(koo_re PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_re SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_re
    PUBLIC
        koo_skills                       # for skill::Workpiece + recognize() entry points
        koo_process                      # for process::ProcessPlan + StepInvocation
        nlohmann_json::nlohmann_json
        TKernel TKMath TKBRep
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_re)
