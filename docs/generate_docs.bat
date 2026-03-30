
breathe-apidoc --generate class --members --force --output-dir doxygen\api_member_list ..\build\docs\doxygen\xml\
breathe-apidoc --generate file --force --output-dir doxygen\api_member_list ..\build\docs\doxygen\xml\
breathe-apidoc --generate struct --members --force --output-dir doxygen\api_member_list ..\build\docs\doxygen\xml\

del doxygen\api_member_list\file\*dox.rst

python dox_converter.py

make.bat html