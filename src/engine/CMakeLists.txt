# @lat: [[architecture/overview#src/engine/]]
add_library(koo_engine STATIC
    PlaceholderCylinder.cpp
    WatchFrontModel.cpp
)
target_include_directories(koo_engine PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_engine SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_engine
    PUBLIC
        TKernel TKMath TKBRep TKTopAlgo TKGeomAlgo
        nlohmann_json::nlohmann_json
    PRIVATE
        spdlog::spdlog
)

koo_apply_compiler_flags(koo_engine)
