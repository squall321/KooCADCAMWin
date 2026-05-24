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

# find_package(OpenCASCADE) — 버전 제약 포함 시도, 실패하면 재시도
find_package(OpenCASCADE 8.0 QUIET)
if (NOT OpenCASCADE_FOUND)
    find_package(OpenCASCADE REQUIRED)
    if (OpenCASCADE_VERSION VERSION_LESS "8.0.0")
        message(FATAL_ERROR
            "OpenCASCADE 8.0.0 or newer is required "
            "(found ${OpenCASCADE_VERSION}). "
            "Set OpenCASCADE_DIR to the OCCT 8.0+ install tree.")
    endif()
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)

# ── LGPL 준수: OCCT 동적 링크 확인 (모듈 함수) ────────────────────────────
# koo_check_occt_dynamic() 는 cmake/CheckOcctDynamic.cmake 에 정의된다.
# 실행 파일 타겟 생성 이후 호출하므로 여기서는 매크로를 등록만 한다.
# (아래 add_executable 이후 호출)

# ── 서브디렉토리 ───────────────────────────────────────────────────────────
add_subdirectory(src/gui)
add_subdirectory(src/engine)
add_subdirectory(src/io)
add_subdirectory(src/dfm)
add_subdirectory(src/re)
add_subdirectory(src/cam)

# ── 메인 실행 파일 ────────────────────────────────────────────────────────
add_executable(KooCADCAM WIN32 src/main.cpp)

target_link_libraries(KooCADCAM PRIVATE
    koo_gui
    koo_engine
    koo_io
    koo_dfm
    koo_re
    koo_cam
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGLWidgets
    TKernel
    TKMath
    TKBRep
    TKGeomAlgo
    TKTopAlgo
    TKV3d
    TKAIS
    TKOpenGl
    TKStep
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

# ── 테스트 ────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
