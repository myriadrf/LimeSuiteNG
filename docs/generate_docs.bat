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
RMDIR /S/Q developer\doxygen\api_member_list\class
RMDIR /S/Q developer\doxygen\api_member_list\file
RMDIR /S/Q developer\doxygen\api_member_list\struct
DEL developer\doxygen\api_member_list\classlist.rst
DEL developer\doxygen\api_member_list\filelist.rst
DEL developer\doxygen\api_member_list\structlist.rst

:: Deleting sphinx manual pages converted from doxygen pages
python dox_converter.py --del

:: If rebuild is not passed, end script
IF "%1"=="" goto :eof

:rebuild
ECHO generate_docs.bat Rebuilding documentation

:: Configuring project
cmake -S .. -B ..\build -G Ninja

:: Building doxygen target
cmake --build ..\build -- doxygen

:: Generating API reference pages for the API reference list
breathe-apidoc --generate class --members --force --output-dir developer\doxygen\api_member_list ..\build\docs\developer\doxygen\xml\
breathe-apidoc --generate file --force --output-dir developer\doxygen\api_member_list ..\build\docs\developer\doxygen\xml\
breathe-apidoc --generate struct --members --force --output-dir developer\doxygen\api_member_list ..\build\docs\developer\doxygen\xml\

:: removing redundant copies of doxygen manual pages
DEL developer\doxygen\api_member_list\file\*dox.rst

:: Converting the actual doxygen manual pages to sphinx .rst format
python dox_converter.py

:: Running sphinx build tool
:sphinx_build
make.bat html

:eof
ENDLOCAL
EXIT /B 0