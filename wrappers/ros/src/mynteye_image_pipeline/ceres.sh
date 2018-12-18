echo "==========Configure and build ceres 1.11.0========"
cd third_party
mkdir -p extend_install/ceres_extend
cd ceres-solver-1.11.0
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../extend_install/ceres_extend -DGFLAGS_LIBRARY_DIR_HINTS=usr/lib/x86_64-linux-gnu -DGFLAGS_PREFER_EXPORTED_GFLAGS_CMAKE_CONFIGURATION=OFF -DMINIGLOG=ON
make -j4
make install
rm -r ../build
