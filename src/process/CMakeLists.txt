# @lat: [[engine/skills#Layer 3 ProcessPlan]]
# Process planning library — Layer 3 on top of koo_skills.
#
# StepInvocation / ProcessPlan: serialisable plan of skill calls.
# Executor: dispatches plan steps to the registered slice-1 skills.

add_library(koo_process STATIC
    ProcessPlan.cpp
    Executor.cpp
)

target_include_directories(koo_process PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_process SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_process
    PUBLIC
        koo_skills                       # for skill::Workpiece, skill::SkillOutput
        nlohmann_json::nlohmann_json
        TKernel TKMath TKBRep
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_process)
