cmake_minimum_required(VERSION 3.27)
project(KooCADCAM VERSION 0.1.0 LANGUAGES CXX)

# ── C++ 표준 ───────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Windows 플랫폼 정의 ────────────────────────────────────────────────────
if (WIN32)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
endif()

# ── Qt 자동화 MOC/UIC/RCC ──────────────────────────────────────────────────
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ── 헬퍼 CMake 모듈 ────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(KooCompilerFlags)
include(CheckOcctDynamic)

# ── vcpkg 통합 (toolchain file 로 활성화됨) ────────────────────────────────
# OCCT 8.0.0 — 소스에서 직접 빌드한 install 트리를 OpenCASCADE_DIR 또는
# CMAKE_PREFIX_PATH 로 지정한다. vcpkg 포트는 사용하지 않는다.

# find_package(OpenCASCADE) — robust against multi-OCCT systems.
# Force search to OCCT_INSTALL_DIR/cmake only; ignore any OCCT 7.x install
# the user may have on PATH or in CMake registry.
if (NOT DEFINED OpenCASCADE_DIR AND DEFINED ENV{OCCT_INSTALL_DIR})
    set(OpenCASCADE_DIR "$ENV{OCCT_INSTALL_DIR}/cmake" CACHE PATH
        "Path to OpenCASCADEConfig.cmake (from OCCT_INSTALL_DIR/cmake)")
endif()
# Normalize Windows backslashes to forward slashes — vcpkg's toolchain wraps
# find_package args in a string parser that rejects backslash escapes such as
# \U (\Users\...), \P (\Program Files\...).  file(TO_CMAKE_PATH) is idempotent
# on Unix-style paths.
if (OpenCASCADE_DIR)
    file(TO_CMAKE_PATH "${OpenCASCADE_DIR}" OpenCASCADE_DIR)
endif()
if (NOT OpenCASCADE_DIR OR NOT EXISTS "${OpenCASCADE_DIR}/OpenCASCADEConfig.cmake")
    message(FATAL_ERROR
        "OpenCASCADE_DIR must point at an OCCT 8.0+ install tree's cmake/ subdir "
        "containing OpenCASCADEConfig.cmake. "
        "Current value: '${OpenCASCADE_DIR}'. "
        "Fix: run scripts/setup-env.ps1 -OcctInstallDir <path-to-occt-8.0-install>, "
        "then open a new shell and rerun the cmake --preset.")
endif()
find_package(OpenCASCADE 8.0.0 REQUIRED CONFIG
    PATHS "${OpenCASCADE_DIR}"
    NO_DEFAULT_PATH
)
if (OpenCASCADE_VERSION VERSION_LESS "8.0.0")
    message(FATAL_ERROR
        "OpenCASCADE 8.0.0 or newer is required (found ${OpenCASCADE_VERSION} at ${OpenCASCADE_DIR}).")
endif()
message(STATUS "KooCADCAM: OpenCASCADE ${OpenCASCADE_VERSION} pinned to ${OpenCASCADE_DIR}")

# Expose OCCT lib dir as a global so cmake/KooCompilerFlags.cmake can inject
# it into every target's link search via target_link_directories.
# (link_directories() at directory scope did not propagate reliably into
# the tests subdir under the vcpkg + Ninja toolchain combo.)
if (WIN32)
    set(_occt_lib_dir "${OpenCASCADE_DIR}/../win64/vc14/lib")
    get_filename_component(_occt_lib_dir "${_occt_lib_dir}" ABSOLUTE)
    if (EXISTS "${_occt_lib_dir}")
        set(KOO_OCCT_LIB_DIR "${_occt_lib_dir}" CACHE INTERNAL
            "OCCT Windows-native lib dir, injected per-target by koo_apply_compiler_flags")
        message(STATUS "KooCADCAM: OCCT lib dir exposed as KOO_OCCT_LIB_DIR: ${_occt_lib_dir}")
    endif()
endif()

# Qt6 — pin to Qt6_DIR if provided (avoids picking a stray system install).
if (NOT DEFINED Qt6_DIR AND DEFINED ENV{Qt6_DIR})
    set(Qt6_DIR "$ENV{Qt6_DIR}/lib/cmake/Qt6" CACHE PATH
        "Path to Qt6Config.cmake (from Qt6_DIR/lib/cmake/Qt6)")
endif()
if (Qt6_DIR)
    file(TO_CMAKE_PATH "${Qt6_DIR}" Qt6_DIR)
endif()
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)
message(STATUS "KooCADCAM: Qt ${Qt6_VERSION} via ${Qt6_DIR}")
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)

# ── LGPL 준수: OCCT 동적 링크 확인 (모듈 함수) ────────────────────────────
# koo_check_occt_dynamic() 는 cmake/CheckOcctDynamic.cmake 에 정의된다.
# 실행 파일 타겟 생성 이후 호출하므로 여기서는 매크로를 등록만 한다.
# (아래 add_executable 이후 호출)

# ── 서브디렉토리 ───────────────────────────────────────────────────────────
add_subdirectory(src/gui)
add_subdirectory(src/engine)
add_subdirectory(src/skills)
add_subdirectory(src/io)
add_subdirectory(src/dfm)
add_subdirectory(src/process)
add_subdirectory(src/adapt)
add_subdirectory(src/re)
add_subdirectory(src/edit)
add_subdirectory(src/cam)
add_subdirectory(src/parts)

# ── 메인 실행 파일 ────────────────────────────────────────────────────────
add_executable(KooCADCAM src/main.cpp)  # WIN32 removed: keep console for log output

target_link_libraries(KooCADCAM PRIVATE
    koo_gui
    koo_engine
    koo_io
    koo_dfm
    koo_process
    koo_adapt
    koo_re
    koo_skills
    koo_cam
    koo_parts
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGLWidgets
    TKernel
    TKMath
    TKBRep
    TKGeomAlgo
    TKTopAlgo
    TKV3d
    TKOpenGl
    TKDESTEP
    TKXCAF
    TKXSBase
    nlohmann_json::nlohmann_json
    spdlog::spdlog
)

# ── 헬퍼 모듈 적용 ────────────────────────────────────────────────────────
koo_apply_compiler_flags(KooCADCAM)
koo_check_occt_dynamic(KooCADCAM)

# ── Qt6 전용 마무리 ────────────────────────────────────────────────────────
if (Qt6_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(KooCADCAM)
endif()

# ── Post-build: deploy Qt + OCCT DLLs next to KooCADCAM.exe ───────────────
# Without this the .exe silently exits on launch when run directly from the
# build dir (missing Qt6Cored.dll, TKernel.dll, etc.).
if (WIN32)
    # 1) Qt: run windeployqt6 to bring in Qt6*d.dll + platforms/iconengines/...
    get_target_property(_qt6_qmake Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(_qt6_bin "${_qt6_qmake}" DIRECTORY)
    find_program(WINDEPLOYQT_EXE NAMES windeployqt6.exe windeployqt.exe
                 HINTS "${_qt6_bin}" NO_DEFAULT_PATH)
    if (WINDEPLOYQT_EXE)
        add_custom_command(TARGET KooCADCAM POST_BUILD
            COMMAND "${WINDEPLOYQT_EXE}" --debug --no-translations
                    --no-system-d3d-compiler --no-opengl-sw
                    "$<TARGET_FILE:KooCADCAM>"
            COMMENT "windeployqt: copying Qt6 DLLs + plugins to $<TARGET_FILE_DIR:KooCADCAM>"
            VERBATIM)
    endif()
    # 2) OCCT: copy TK*.dll next to the exe (OCCT install is in a non-standard
    #    win64/vc14/bin path that doesn't get auto-resolved at runtime).
    add_custom_command(TARGET KooCADCAM POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
                "${KOO_OCCT_LIB_DIR}/../bin"
                "$<TARGET_FILE_DIR:KooCADCAM>"
        COMMENT "copying OCCT TK*.dll to $<TARGET_FILE_DIR:KooCADCAM>"
        VERBATIM)
endif()

# ── 테스트 ────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
