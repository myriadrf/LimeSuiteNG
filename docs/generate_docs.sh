#!/bin/bash
## Must be run from within the venv.

# Usage:
# ./generate_docs.sh - to just regenerate sphinx docs
# ./generate_docs.sh clean - to clean all the build files
# ./generate_docs.sh rebuild - to rebuild the sphinx with new build files
# ./generate_docs.sh clean rebuild - to clean the build files and rebuild with new build files

set -e

if [[ ! -d "_build" || ! -d "apidoc" || ! -d "../build/xml" ]]; then
    set "rebuild"
fi

if [[ $1 == "clean" ]]; then
    echo "Deleting sphinx _build folder"
    rm -rf _build

    echo "Deleting API reference pages"
    rm -rf doxygen/api_member_list/class
    rm -rf doxygen/api_member_list/file
    rm -rf doxygen/api_member_list/struct
    rm doxygen/api_member_list/classlist.rst
    rm doxygen/api_member_list/filelist.rst
    rm doxygen/api_member_list/structlist.rst    

    echo "Deleting doxygen manual pages"
    python dox_converter.py --del

    shift
    if [[ $1 == "" ]]; then
        exit 0
    fi
fi

if [[ $1 == "rebuild" ]]; then
    cmake -S .. -B ../build
    cmake --build ../build -- doxygen
    breathe-apidoc --generate class --members --force --output-dir doxygen/api_member_list ../build/docs/doxygen/xml/
    breathe-apidoc --generate file --force --output-dir doxygen/api_member_list ../build/docs/doxygen/xml/
    breathe-apidoc --generate struct --members --force --output-dir doxygen/api_member_list ../build/docs/doxygen/xml/
    rm doxygen/api_member_list/file/*dox.rst
    python dox_converter.py
fi

make -j"$(nproc)" html
