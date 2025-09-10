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
    -DINSTALL_DEVELOPMENT=ON ^
    -DBUILD_PLUGINS=ON ^
    ..
if errorlevel 1 exit 1

:: build
cmake --build . --config Release
if errorlevel 1 exit 1

:: install
cmake --install . --config Release
if errorlevel 1 exit 1

:: Test gnuradio-limesuiteng python lib import
python -c "import gnuradio.limesuiteng"
if errorlevel 1 exit 1