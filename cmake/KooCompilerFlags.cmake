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
endfunction()
