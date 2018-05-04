cmake_minimum_required(VERSION 3.0)

project(mynteye_test VERSION 2.0.0 LANGUAGES C CXX)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)

# flags

if(OS_WIN)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include(${PRO_DIR}/cmake/DetectCXX11.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

#if(OS_WIN AND NOT MINGW)
#  set(CompilerFlags
#    CMAKE_CXX_FLAGS
#    CMAKE_CXX_FLAGS_DEBUG
#    CMAKE_CXX_FLAGS_RELEASE
#    CMAKE_C_FLAGS
#    CMAKE_C_FLAGS_DEBUG
#    CMAKE_C_FLAGS_RELEASE
#  )
#  foreach(CompilerFlag ${CompilerFlags})
#    string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
#  endforeach()
#endif()

string(STRIP "${CMAKE_C_FLAGS}" CMAKE_C_FLAGS)
string(STRIP "${CMAKE_CXX_FLAGS}" CMAKE_CXX_FLAGS)
message(STATUS "C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# packages

LIST(APPEND CMAKE_PREFIX_PATH ${PRO_DIR}/third_party/glog/_build)
find_package(glog REQUIRED)
message(STATUS "Found glog: ${glog_VERSION}")

LIST(APPEND CMAKE_PREFIX_PATH ${PRO_DIR}/_install/lib/cmake)
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

## gtest

set(TEST_DIR ${PROJECT_SOURCE_DIR})
set(GMOCK_DIR ${TEST_DIR}/gtest/googlemock)
set(GTEST_DIR ${TEST_DIR}/gtest/googletest)

set(GMOCK_LIB_DIR ${TEST_DIR}/gtest/_build/googlemock)
set(GTEST_LIB_DIR ${GMOCK_LIB_DIR}/gtest)
if(OS_WIN AND NOT MINGW)
  set(GTEST_LIB_DIR ${GTEST_LIB_DIR}/Release)
endif()

set(GTEST_LIBS
  ${GTEST_LIB_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
  #${GTEST_LIB_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
)
message(STATUS "Found gtest libs: ${GTEST_LIBS}")

# targets

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")

set_outdir(
  "${OUT_DIR}/lib"
  "${OUT_DIR}/lib"
  "${OUT_DIR}/bin"
)

include_directories(
  ${GTEST_DIR}/include
  ${PRO_DIR}/src
)

file(GLOB TEST_INTERNAL_SRC "internal/*.cc")
file(GLOB TEST_PUBLIC_SRC "public/*.cc")

add_executable(${PROJECT_NAME}
  gtest_main.cc
  ${TEST_INTERNAL_SRC}
  ${TEST_PUBLIC_SRC}
)
target_link_libraries(${PROJECT_NAME} mynteye ${GTEST_LIBS} ${OpenCV_LIBS})
target_create_scripts(${PROJECT_NAME} DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${OpenCV_LIB_SEARCH_PATH}
)

if(OS_WIN)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

#find_package(Threads REQUIRED)
#if(THREADS_HAVE_PTHREAD_ARG)
#  target_compile_options(PUBLIC ${PROJECT_NAME} "-pthread")
#endif()
#if(CMAKE_THREAD_LIBS_INIT)
#  target_link_libraries(${PROJECT_NAME} "${CMAKE_THREAD_LIBS_INIT}")
#endif()
