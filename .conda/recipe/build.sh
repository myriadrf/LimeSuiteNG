#!/usr/bin/env bash

cmake -E make_directory buildconda
cd buildconda

cmake_config_args=(
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=$PREFIX
    -DCMAKE_PREFIX_PATH=$PREFIX
    -DGR_PYTHON_DIR=$SP_DIR
    -DLIB_SUFFIX=""
    -DENABLE_DOXYGEN=OFF
    -DENABLE_TESTING=ON
    -DCMAKE_POLICY_VERSION_MINIMUM=3.15
    -DINSTALL_DEVELOPMENT=ON
    -DBUILD_PLUGINS=OFF
    -DGUI_TOGGLE=OFF
    -DBUILD_DRIVERS=OFF
    -DCONDA_UDEV_RULES=ON
    -DUDEV_RULES_RELOAD_ON_INSTALL=OFF
)

cmake ${CMAKE_ARGS} -G "Ninja" "${cmake_config_args[@]}" ..
cmake --build . --config Release -- -j$((CPU_COUNT-2))
cmake --build . --config Release --target install
