@ECHO off
SETLOCAL
:: conda_deps.bat script installs appropriate conda packages required to build LimeSuiteNG project from source in conda environment.

:: Set or update any variables here
SET CONDA_DEPS= 
SET CONDA_B_PKG_NAME=conda-build
SET CONDA_F_PKG_NAME=conda-forge-pinning
SET VS_PKG_NAME=vs2022_win-64
SET CMAKE_PKG_NAME=cmake
SET NINJA_PKG_NAME=ninja
SET ENV_MIN_VS_DISP_VER="17.14.7"
SET MIN_VS_LINE_VER=2022


:: Script start
ECHO # - Starting LimeSuiteNG conda package installation
WHERE /Q conda
IF ERRORLEVEL 1 (
   ECHO # - Error: Using invalid prompt^! Use radioconda prompt with admin privileges^!
   EXIT /B   
)

FOR /f "delims=" %%i in ('conda info ^| findstr /C:"active environment"') do SET CURR_CONDA_ENV=%%i
SET "CURR_CONDA_ENV=%CURR_CONDA_ENV:*active environment : =%"

IF "%CURR_CONDA_ENV%"=="base" (
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

FOR /f %%i in ('conda list %CONDA_B_PKG_NAME% ^| findstr /C:"%CONDA_B_PKG_NAME%"') do SET CONDA_BUILD_PKG=%%i
IF NOT "%CONDA_BUILD_PKG%"=="%CONDA_B_PKG_NAME%" (
   ECHO # - %CONDA_B_PKG_NAME% tools package missing! Adding package to dependency list.
   SET "CONDA_DEPS=%CONDA_DEPS% %CONDA_B_PKG_NAME%"
) ELSE (
   ECHO # - %CONDA_B_PKG_NAME% package detected. Skipping.
)

FOR /f %%i in ('conda list %CONDA_F_PKG_NAME% ^| findstr "%CONDA_F_PKG_NAME%"') do SET CONDA_FORGE_PKG=%%i
IF NOT "%CONDA_FORGE_PKG%"=="%CONDA_F_PKG_NAME%" (
   ECHO # - %CONDA_F_PKG_NAME% package missing^! Adding package to dependency list^!
   SET "CONDA_DEPS=%CONDA_DEPS% %CONDA_F_PKG_NAME%"
) ELSE (
   ECHO # - %CONDA_F_PKG_NAME% package detected. Skipping.
)

FOR /f %%i in ('conda list %VS_PKG_NAME% ^| findstr /C:"%VS_PKG_NAME%"') do SET VS2022_BAT=%%i
IF NOT "%VS2022_BAT%"=="%VS_PKG_NAME%" (
   ECHO # - %VS_PKG_NAME% package missing^! Adding package to dependency list^!
   SET "CONDA_DEPS=%CONDA_DEPS% %VS_PKG_NAME%"
) ELSE (
   ECHO # - %VS_PKG_NAME% package detected. Skipping.
)

FOR /f %%i in ('conda list %CMAKE_PKG_NAME% ^| findstr /C:"%CMAKE_PKG_NAME%"') do SET CMAKE_PKG=%%i
IF NOT "%CMAKE_PKG%"=="%CMAKE_PKG_NAME%" (
   ECHO # - %CMAKE_PKG_NAME% package missing^! Adding package to dependency list^!
   SET "CONDA_DEPS=%CONDA_DEPS% %CMAKE_PKG_NAME%"
) ELSE (
   ECHO # - %CMAKE_PKG_NAME% package detected. Skipping.
)

FOR /f %%i in ('conda list %NINJA_PKG_NAME% ^| findstr /C:"%NINJA_PKG_NAME%"') do SET NINJA_PKG=%%i
IF NOT "%NINJA_PKG%"=="%NINJA_PKG_NAME%" (
   ECHO # - %NINJA_PKG_NAME% package missing^! Adding package to dependency list^!
   SET "CONDA_DEPS=%CONDA_DEPS% %NINJA_PKG_NAME%"
) ELSE (
   ECHO # - %NINJA_PKG_NAME% package detected. Skipping.
)

@REM SET "CONDA_DEPS=%CONDA_DEPS% zstd g++ gcc"
IF "%CONDA_DEPS%"==" " (
   ECHO # - ==================================================================================
   ECHO # -   All conda packages are present^! Skipping LimeSuiteNG component installation^!
   ECHO # - ==================================================================================
   GOTO finalize
   )

ECHO # - ==============================================================
ECHO # -   Installing following conda packages for LimeSuiteNG build:
ECHO # - ==============================================================
FOR %%i in (%CONDA_DEPS%) do (
   ECHO # -    %%i
)
ECHO # -

CALL conda install %CONDA_DEPS%

:finalize

:: Check for Visual Studio Build Tools installation
ECHO # - ========================================================
ECHO # -   Checking for Visual Studio Build Tools 2022 package
ECHO # - ========================================================

FOR /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VS_INSTALL_DIR=%%i
FOR /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property productLineVersion`) do set VS_LINE_VER=%%i
FOR /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property productDisplayVersion`) do set VS_DISP_VER=%%i

IF "%VS_INSTALL_DIR%"=="" (
   ECHO # - Visual Studio Build Tools are not installed^! Please install Visual Studio Build Tools 2022 from official microsoft visual studio page^!
) ELSE (
   ECHO # - Detected Visual Studio Build Tools.
   ECHO # - Installation path: "%VS_INSTALL_DIR%"

   IF "%VS_LINE_VER%"=="%MIN_VS_LINE_VER%" (
      ECHO # - Product line version: %VS_LINE_VER% ^(Correct^)
      ECHO # - Product display version: %VS_DISP_VER% ^(Recommended minimum version: %ENV_MIN_VS_DISP_VER%^)
      ECHO # - Warning: Make sure you have installed the latest or atleast recommended minimum version of Visual Studio Build Tools.
   ) ELSE (
      ECHO # - Product line version: %VS_LINE_VER% ^(Incorrect^)
      ECHO # - Warning: Install the correct product line version.
   )
)
ECHO # - Exiting conda_deps.bat script
ENDLOCAL