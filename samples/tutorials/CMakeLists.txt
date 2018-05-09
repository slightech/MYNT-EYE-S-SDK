get_filename_component(DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

set_outdir(
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/bin/${DIR_NAME}"
)

# make_executable(NAME
#                 [SRCS src1 src2 ...])
#                 [WITH_OPENCV])
macro(make_executable NAME)
  set(options WITH_OPENCV)
  set(oneValueArgs)
  set(multiValueArgs SRCS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  set(__link_libs mynteye)
  set(__dll_search_paths ${PRO_DIR}/_install/bin)
  if(THIS_WITH_OPENCV)
    list(APPEND __link_libs ${OpenCV_LIBS})
    list(APPEND __dll_search_paths ${OpenCV_LIB_SEARCH_PATH})
  endif()

  add_executable(${NAME} ${THIS_SRCS})
  target_link_libraries(${NAME} ${__link_libs})
  target_create_scripts(${NAME} DLL_SEARCH_PATHS ${__dll_search_paths})

  if(OS_WIN)
    target_compile_definitions(${NAME}
      PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
    )
  endif()
endmacro()

if(WITH_API)

make_executable(get_device_info SRCS data/get_device_info.cc)
make_executable(get_with_plugin SRCS data/get_with_plugin.cc)

make_executable(ctrl_infrared SRCS control/infrared.cc WITH_OPENCV)

endif()
