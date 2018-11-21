@echo off
setlocal
set _MY_DIR=%~dp0

set _VS_GEN="Visual Studio 15 2017 Win64"
REM set _VS_GEN="Visual Studio 14 2015 Win64"

cd %_MY_DIR%

mkdir _build
cd _build/

cmake -DCMAKE_BUILD_TYPE=Release ^
-DCMAKE_PREFIX_PATH="%_MY_DIR%/../lib/cmake" ^
-DOpenCV_DIR="%_MY_DIR%/../3rdparty/opencv/build" ^
-G %_VS_GEN% ^
..

cd %_MY_DIR%
pause
