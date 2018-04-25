get_filename_component(DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

set_outdir(
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/bin/${DIR_NAME}"
)

include_directories(
  ${PRO_DIR}/src
)

## camera_u

add_executable(camera_u camera.cc)
target_link_libraries(camera_u mynteye ${OpenCV_LIBS})
target_create_scripts(camera_u DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${OpenCV_LIB_SEARCH_PATH}
)

if(OS_WIN)
  target_compile_definitions(camera_u
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()
