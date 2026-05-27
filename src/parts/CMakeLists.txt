# @lat: [[engine/skills#Layer 6 Parts layout]]
# Parts layout + datum-part dependency graph (Layer 6).
#
# This library captures the *positions* of internal parts (PCB, battery,
# display, ...) and the *dependency edges* between ProcessPlan step indices
# and those parts.  When a part moves or resizes, DatumGraph::invalidatedSteps
# tells the Layer-5 adapter which steps need re-evaluation.
#
# Depends on:
#   - koo_io       → StepIO for loading parts from STEP files.
#   - koo_process  → ProcessPlan (used by the typed
#                    extractHeuristicDependencies overload).  If
#                    koo_process is unavailable in a stripped-down build,
#                    DatumGraph.hpp falls back to the json-vector overload
#                    via __has_include guard.
#   - OCCT TKernel/TKMath/TKBRep — for TopoDS_Shape, Bnd_Box, gp_Trsf.
#   - nlohmann_json — for the diff details + JSON serialisation.

add_library(koo_parts STATIC
    PartsLayout.cpp
    DatumGraph.cpp
)

target_include_directories(koo_parts PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_parts SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_parts
    PUBLIC
        koo_io                           # StepIO
        koo_process                      # process::ProcessPlan
        nlohmann_json::nlohmann_json
        TKernel TKMath TKBRep
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_parts)
