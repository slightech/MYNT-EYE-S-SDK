get_filename_component(DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

set_outdir(
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/bin/${DIR_NAME}"
)

include_directories(
  ${PRO_DIR}/src
)

## record

add_executable(record record.cc)
target_link_libraries(record mynteye ${OpenCV_LIBS})

if(OS_WIN)
  target_compile_definitions(record
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()
