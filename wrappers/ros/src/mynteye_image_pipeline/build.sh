echo "Configure and building third party/eigen 3.2.10"
cd third_party
mkdir -p extend_install/eigen_extend
cd eigen-eigen-b9cd8366d4e8
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../extend_install/eigen_extend
make install
rm -r ../build
echo "==========Configure and build ceres 1.11.0========"
cd ../../
mkdir -p extend_install/ceres_extend
cd ceres-solver-1.11.0
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../extend_install/ceres_extend
make -j4
make install
rm -r ../build
