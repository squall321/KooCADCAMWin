cmake_minimum_required(VERSION 3.27)
include_guard(GLOBAL)

# CheckOcctDynamic.cmake
# Verifies that all TK* OCCT libraries linked to a target are SHARED.
# Enforces LGPL compliance per lat.md architecture/build-and-deps.md.
# Usage: koo_check_occt_dynamic(<target>)

function(koo_check_occt_dynamic target)
    get_target_property(_libs ${target} LINK_LIBRARIES)

    if(NOT _libs)
        # No link libraries — nothing to check.
        return()
    endif()

    foreach(_lib IN LISTS _libs)
        # Only inspect targets whose name starts with TK.
        if(NOT _lib MATCHES "^TK")
            continue()
        endif()

        # The library name might be a CMake target or a raw path.
        if(NOT TARGET ${_lib})
            # Raw path or system lib — skip; can't inspect TYPE.
            continue()
        endif()

        get_target_property(_type ${_lib} TYPE)

        if(NOT _type STREQUAL "SHARED_LIBRARY")
            message(FATAL_ERROR
                "LGPL violation: ${_lib} is not dynamically linked "
                "(TYPE=${_type}). Rebuild OCCT with "
                "-DBUILD_LIBRARY_TYPE=Shared."
            )
        endif()
    endforeach()
endfunction()
