setlocal EnableDelayedExpansion
@echo on

:: Make a build folder and change to it
cmake -E make_directory buildconda
cd buildconda

:: configure
cmake -G "Ninja" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
    -DCMAKE_PREFIX_PATH="%LIBRARY_PREFIX%" ^
    -DGR_PYTHON_DIR="%SP_DIR%" ^
    -DENABLE_DOXYGEN=OFF ^
    -DENABLE_TESTING=ON ^
    -DCMAKE_POLICY_VERSION_MINIMUM=3.15 ^
    ../plugins/gr-limesuiteng

if errorlevel 1 exit 1

:: build
cmake --build . --config Release -- -j%CPU_COUNT%
if errorlevel 1 exit 1

:: install
cmake --build . --config Release --target install
if errorlevel 1 exit 1

:: Test
python -c "import gnuradio.limesuiteng"
if errorlevel 1 exit 1