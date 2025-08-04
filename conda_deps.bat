@ECHO off
SETLOCAL ENABLEDELAYEDEXPANSION
:: conda_deps.bat script installs appropriate conda packages required to build LimeSuiteNG project from source in conda environment.

:: Set or update any variables here
SET CONDA_DEPS= 
SET CONDA_B_PKG_NAME=conda-build
SET CONDA_F_PKG_NAME=conda-forge-pinning
SET VS_PKG_NAME=vs2022_win-64
SET CMAKE_PKG_NAME=cmake
SET NINJA_PKG_NAME=ninja
SET GNURADIO_PKG_NAME=gnuradio
SET BOOST_PKG_NAME=boost
SET ENV_MIN_VS_DISP_VER="17.14.7"
SET MIN_VS_LINE_VER=2022

:: GNURadio version limitations
SET GNUR_MAJOR_VER=3
SET GNUR_MINOR_VER=10

SET GNURADIO_VERSION_FLAG=--v

:: Script start
ECHO # - Starting LimeSuiteNG conda package installation
WHERE /Q conda
IF ERRORLEVEL 1 (
   ECHO # - Error: Using invalid prompt. Use radioconda prompt with admin privileges.
   EXIT /B   
)

FOR /f "delims=" %%i in ('conda info ^| findstr /C:"active environment"') do SET CURR_CONDA_ENV=%%i
SET "CURR_CONDA_ENV=!CURR_CONDA_ENV:*active environment : =!"

IF "!CURR_CONDA_ENV!"=="base" (
   ECHO # - Error: Trying to install conda packages into base environment!
   ECHO # - Activate your environment:
   ECHO # -
   ECHO # - conda activate ^<your custom env name^>
   ECHO # -
   EXIT /B
)

ECHO # - ========================================================
ECHO # -   Checking for installed packages in conda environment
ECHO # - ========================================================

CALL :CheckForPackage %CONDA_B_PKG_NAME%
CALL :CheckForPackage %CONDA_F_PKG_NAME%
CALL :CheckForPackage %VS_PKG_NAME%
CALL :CheckForPackage %CMAKE_PKG_NAME%
CALL :CheckForPackage %NINJA_PKG_NAME%

:: Check if gnuradio-limesuiteng plugin dependencies are requested (was '--v' flag passed)
IF "%1"=="%GNURADIO_VERSION_FLAG%" (
   ECHO # - ================================================================================
   ECHO # -   Checking for additional conda packages for gnuradio-limesuiteng plugin build
   ECHO # - ================================================================================
) ELSE (
   GOTO skipPluginDeps
)
ECHO # - Checking if requested GNURadio version is valid.
SET REQ_VER=%2
CALL :CheckReqGnuradioVer !REQ_VER!
IF !ERRORLEVEL! NEQ 0 (
   CALL :VersionCheckError !ERRORLEVEL!
   GOTO ending
)

FOR /f %%i in ('conda list %GNURADIO_PKG_NAME% ^| findstr /C:"%GNURADIO_PKG_NAME%"') do SET GNURADIO_PKG=%%i
IF NOT DEFINED GNURADIO_PKG (
   ECHO # - %GNURADIO_PKG_NAME% package missing. Adding package to dependency list.
   SET "CONDA_DEPS=!CONDA_DEPS! %GNURADIO_PKG_NAME%="
   CALL :LoadGNURadioDeps !REQ_VER!
   IF !ERRORLEVEL! NEQ 0 GOTO ending
   SET "CONDA_DEPS=!CONDA_DEPS!!PLUGIN_DEPS!"
) ELSE (
   ECHO # - %GNURADIO_PKG_NAME% package detected. Skipping.
)

:skipPluginDeps

IF "!CONDA_DEPS!"==" " (
   ECHO # - ==================================================================================
   ECHO # -   All conda packages are present. Skipping LimeSuiteNG component installation.
   ECHO # - ==================================================================================
   GOTO finalize
   )

ECHO # - ==============================================================
ECHO # -   Installing following conda packages for LimeSuiteNG build:
ECHO # - ==============================================================
FOR %%i in (!CONDA_DEPS!) do (
   ECHO # -    %%i
)
ECHO # -

CALL conda install !CONDA_DEPS!

:finalize

:: Check for Visual Studio Build Tools installation
ECHO # - ========================================================
ECHO # -   Checking for Visual Studio Build Tools 2022 package
ECHO # - ========================================================

FOR /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VS_INSTALL_DIR=%%i
FOR /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property productLineVersion`) do set VS_LINE_VER=%%i
FOR /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property productDisplayVersion`) do set VS_DISP_VER=%%i

IF NOT DEFINED VS_INSTALL_DIR (
   ECHO # - Visual Studio Build Tools are not installed. Please install Visual Studio Build Tools 2022 from official microsoft visual studio page.
) ELSE (
   ECHO # - Detected Visual Studio Build Tools.
   ECHO # - Installation path: "!VS_INSTALL_DIR!"

   IF "!VS_LINE_VER!"=="%MIN_VS_LINE_VER%" (
      ECHO # - Product line version: !VS_LINE_VER! ^(Correct^)
      ECHO # - Product display version: !VS_DISP_VER! ^(Recommended minimum version: %ENV_MIN_VS_DISP_VER%^)
      ECHO # - Warning: Make sure you have installed the latest or atleast recommended minimum version of Visual Studio Build Tools.
   ) ELSE (
      ECHO # - Product line version: !VS_LINE_VER! ^(Incorrect^)
      ECHO # - Warning: Install the correct product line version.
   )
)
:ending
ECHO # - Exiting conda_deps.bat script
ENDLOCAL
EXIT /B 0

:: =======================
::       SUB-ROUTINES
:: =======================


:: :CheckForPackage 

:: Checks if the requested package is already installed in conda environment.
:: If not, append the package name to dependency list.
:: Arguments:
:: %1 - Requested package name

:CheckForPackage
   set REQ_PKG=%1
   FOR /f %%i in ('conda list !REQ_PKG! ^| findstr /C:"!REQ_PKG!"') do SET PKG_STATUS=%%i
   IF NOT DEFINED PKG_STATUS (
      ECHO # - !REQ_PKG! package missing! Adding package to dependency list.
      SET "CONDA_DEPS=!CONDA_DEPS! !REQ_PKG!"
   ) ELSE (
      ECHO # - !REQ_PKG! package detected. Skipping.
   )
   EXIT /B

:: :CheckReqGnuradioVer

:: Check if the requested GNURadio version is valid.
:: Arguments:
:: %1 - Version string
:: ErrorLevels:
:: 0 - Requested GNURadio version valid.
:: 1 - GNURadio version not supported.
:: 2 - Major GNURadio version not supported.
:: 3 - Minor GNURadio version not supported.

:CheckReqGnuradioVer

   SET VER_STR=%~1
   :: Strip the version string into 4 variables
   FOR /F "tokens=1-4 delims=." %%A IN ("%VER_STR%") DO (
      SET "REQ_MAJOR=%%A"
      SET "REQ_MINOR=%%B"
      SET "REQ_PATCH=%%C"
      SET "REQ_FIX=%%D"
   )

   IF NOT DEFINED REQ_MAJOR EXIT /B 1
   IF NOT DEFINED REQ_MINOR EXIT /B 1
   IF NOT DEFINED REQ_PATCH EXIT /B 1
   IF NOT DEFINED REQ_FIX EXIT /B 1

   IF !REQ_MAJOR! NEQ %GNUR_MAJOR_VER% EXIT /B 2
   IF !REQ_MINOR! NEQ %GNUR_MINOR_VER% EXIT /B 3

   FOR /f "delims=" %%i in ('conda search gnuradio ^| findstr /C:"!VER_STR!"') do SET FOUND_VER=%%i
   IF NOT DEFINED FOUND_VER EXIT /B 1

   EXIT /B 0

:: :VersionCheckError

:: Prints error message if passed GNURadio version is not valid.

:VersionCheckError
   IF %1 EQU 1 ECHO # - Error: Passed GNURadio version not supported.
   IF %1 EQU 2 ECHO # - Error: Passed GNURadio major version incorrect. Allowed major version 3.
   IF %1 EQU 3 ECHO # - Error: Passed GNURadio minor version incorrect. Allowed minor version 10.
   EXIT /B 0

:: :LoadGNURadioDeps

:: Loads GNURadio dependencies from a file acording to requested GNURadio version
:: ErrorLevels:
:: 0 - Dependencies found.
:: 1 - Missing dependencies.

:LoadGNURadioDeps

   SET REQ_VER=%1
   FOR /f "delims=" %%i in ('findstr /C:!REQ_VER! conda_requirements.txt') do SET PLUGIN_DEPS=%%i
   IF NOT DEFINED PLUGIN_DEPS (
      ECHO # - Error: Cannot find required dependencies for selected GNURadio version. Update conda_requirements.txt file.
      EXIT /B 1
   )
   EXIT /B 0
