# @lat: [[architecture/overview#src/gui/]]
add_library(koo_gui STATIC
    MainWindow.cpp
    OcctViewWidget.cpp
    AppMenus.cpp
    ParameterPanel.cpp
)
target_include_directories(koo_gui PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(koo_gui SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(koo_gui
    PUBLIC
        Qt6::Core Qt6::Widgets Qt6::OpenGLWidgets
        TKernel TKMath TKBRep TKV3d TKOpenGl TKTopAlgo
        nlohmann_json::nlohmann_json
    PRIVATE
        koo_engine
        koo_io
        spdlog::spdlog
)

set_target_properties(koo_gui PROPERTIES AUTOMOC ON)
koo_apply_compiler_flags(koo_gui)
