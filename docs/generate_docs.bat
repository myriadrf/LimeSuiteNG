@ECHO off
SETLOCAL ENABLEDELAYEDEXPANSION

:: Must be run from within python virtual environment and inside docs directory. 
:: Usage:
:: ./generate_docs.sh - to just regenerate the main docs
:: ./generate_docs.sh clean - to clean all the build files
:: ./generate_docs.sh rebuild - to rebuild the Doxygen definitions
:: ./generate_docs.sh clean rebuild - to clean the build files and rebuild the definitions

IF "%1" EQU "clean" goto :clean
IF "%1" EQU "rebuild" goto :rebuild
IF "%1" EQU "" goto :sphinx_build

:clean

:: Shift cmd line args to check for other flags
SHIFT
ECHO generate_docs.bat: Cleaning build artifacts
:: Deleting sphinx build files
RMDIR /S/Q _build 

:: Deleting breathe-apidoc extension generated files
RMDIR /S/Q doxygen\api_member_list\class
RMDIR /S/Q doxygen\api_member_list\file
RMDIR /S/Q doxygen\api_member_list\struct
DEL doxygen\api_member_list\classlist.rst
DEL doxygen\api_member_list\filelist.rst
DEL doxygen\api_member_list\structlist.rst

:: Deleting sphinx manual pages converted from doxygen pages
python dox_converter.py --del

:: If rebuild is not passed, end script
IF "%1"=="" goto :eof

:rebuild
ECHO generate_docs.bat Rebuilding documentation
cmake -S .. -B ..\build -G Ninja
cmake --build ..\build -- doxygen

breathe-apidoc --generate class --members --force --output-dir doxygen\api_member_list ..\build\docs\doxygen\xml\
breathe-apidoc --generate file --force --output-dir doxygen\api_member_list ..\build\docs\doxygen\xml\
breathe-apidoc --generate struct --members --force --output-dir doxygen\api_member_list ..\build\docs\doxygen\xml\

DEL doxygen\api_member_list\file\*dox.rst

python dox_converter.py

:sphinx_build
make.bat html

:eof
ENDLOCAL
EXIT /B 0