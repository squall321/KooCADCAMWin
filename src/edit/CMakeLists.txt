# @lat: [[engine/reverse-route#feature-editor]]
add_library(koo_edit STATIC
    FeatureEditor.cpp
)
target_include_directories(koo_edit PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_edit SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_edit
    PUBLIC
        TKernel TKMath TKBRep TKTopAlgo
        nlohmann_json::nlohmann_json
    PRIVATE
        koo_engine
        TKBO TKBool TKPrim TKShHealing TKFeat
)

koo_apply_compiler_flags(koo_edit)
