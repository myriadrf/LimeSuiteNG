#!/usr/bin/env bash

cmake -E make_directory buildconda
cd buildconda

# Bypassing conda auto script install to metapackage package and moving the scripts to liblimesuiteng sub-package that contains udev rules. 
mv $SRC_DIR/.conda/metapkg/post-link.sh $PREFIX/bin/.liblimesuiteng-post-link.sh
mv $SRC_DIR/.conda/metapkg/pre-unlink.sh $PREFIX/bin/.liblimesuiteng-pre-unlink.sh

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
    -DBUILD_PLUGINS=ON
    -DBUILD_GUI=OFF
    -DBUILD_DRIVERS=OFF
    -DUDEV_RULES_INSTALL_PATH="${PREFIX}/etc/udev/rules.d"
    -DUDEV_RULES_RELOAD_ON_INSTALL=OFF
)

cmake ${CMAKE_ARGS} -G "Ninja" "${cmake_config_args[@]}" ..
cmake --build . --config Release -- -j$((CPU_COUNT-1))
cmake --build . --config Release --target install
