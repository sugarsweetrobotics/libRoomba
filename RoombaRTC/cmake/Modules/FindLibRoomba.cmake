find_package(PkgConfig)
#pkg_check_modules(PC_OPENRTM openrtm-aist)
#pkg_check_modules(PC_COIL libcoil)

find_path(LIBROOMBA_INCLUDE_DIR libroomba.h
    HINTS ${LIBROOMBA_ROOT}/include $ENV{LIBROOMBA_ROOT}/include /usr/local/share/libroomba/include
    ${PC_LIBROOMBA_INCLUDE_DIRS})
find_library(LIBROOMBA_LIBRARY Roomba
    HINTS ${LIBROOMBA_ROOT}/lib /usr/lib
    ${PC_LIBROOMBA_LIBRARY_DIRS})

set(LIBROOMBA_CFLAGS ${PC_LIBROOMBA_CFLAGS_OTHER})
set(LIBROOMBA_INCLUDE_DIRS ${LIBROOMBA_INCLUDE_DIR})
set(LIBROOMBA_LIBRARIES ${LIBROOMBA_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libRoomba
    REQUIRED_VARS LIBROOMBA_INCLUDE_DIR LIBROOMBA_LIBRARY)


#macro(_IDL_OUTPUTS _idl _dir _result)
#    set(${_result} ${_dir}/${_idl}SK.cc ${_dir}/${_idl}.hh
#        ${_dir}/${_idl}DynSK.cc)
#endmacro(_IDL_OUTPUTS)


#macro(_COMPILE_IDL _idl_file)
#    get_filename_component(_idl ${_idl_file} NAME_WE)
#    set(_idl_srcs_var ${_idl}_SRCS)
#    _IDL_OUTPUTS(${_idl} ${CMAKE_CURRENT_BINARY_DIR} ${_idl_srcs_var})#
#
#    add_custom_command(OUTPUT ${${_idl_srcs_var}}
#        COMMAND ${OPENRTM_IDL_COMPILER} ${OPENRTM_IDL_FLAGS}
#        -I${OPENRTM_IDL_DIR} ${_idl_file}
#        WORKING_DIRECTORY ${CURRENT_BINARY_DIR}
#        DEPENDS ${_idl_file}
#        COMMENT "Compiling ${_idl_file}" VERBATIM)
#    add_custom_target(${_idl}_TGT DEPENDS ${${_idl_srcs_var}})
#    set(ALL_IDL_SRCS ${ALL_IDL_SRCS} ${${_idl_srcs_var}})
#    if(NOT TARGET ALL_IDL_TGT)
#        add_custom_target(ALL_IDL_TGT)
#    endif(NOT TARGET ALL_IDL_TGT)
#    add_dependencies(ALL_IDL_TGT ${_idl}_TGT)
#endmacro(_COMPILE_IDL)

# Module exposed to the user
#macro(OPENRTM_COMPILE_IDL_FILES)
#    foreach(idl ${ARGN})
#        _COMPILE_IDL(${idl})
#    endforeach(idl)
#endmacro(OPENRTM_COMPILE_IDL_FILES)

