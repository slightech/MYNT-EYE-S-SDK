get_filename_component(DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

set_outdir(
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/bin/${DIR_NAME}"
)

include_directories(
  ${PRO_DIR}/src
)

## camera

add_executable(camera camera.cc)
target_link_libraries(camera mynteye ${OpenCV_LIBS})

if(OS_WIN)
  target_compile_definitions(camera
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()
