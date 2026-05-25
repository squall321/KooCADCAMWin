cmake_minimum_required(VERSION 3.27)
include_guard(GLOBAL)

# KooCompilerFlags.cmake
# Applies project-wide compiler flags to a target.
# Usage: koo_apply_compiler_flags(<target>)

function(koo_apply_compiler_flags target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            /WX
            /permissive-
            /utf-8
            /Zc:__cplusplus
            /MP
        )
        target_compile_definitions(${target} PRIVATE
            _CRT_SECURE_NO_WARNINGS
            NOMINMAX
        )
    else()
        # GCC / Clang
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror
            -Wno-unknown-pragmas
            -Wno-deprecated-declarations
            -Wshadow
            -Wnon-virtual-dtor
            -Wold-style-cast
        )
    endif()

    target_compile_features(${target} PRIVATE cxx_std_17)

    # Some OCCT 8.0 imported targets (e.g. TKStep) ship without
    # IMPORTED_LOCATION in their config files, so link.exe sees bare
    # "TKStep.lib" with no search path. Inject the Windows-native OCCT
    # lib dir as a PRIVATE link path so bare names resolve.
    # KOO_OCCT_LIB_DIR is set by the top-level CMakeLists.txt after
    # find_package(OpenCASCADE).
    if (WIN32 AND DEFINED KOO_OCCT_LIB_DIR AND EXISTS "${KOO_OCCT_LIB_DIR}")
        target_link_directories(${target} PRIVATE "${KOO_OCCT_LIB_DIR}")
    endif()
endfunction()
