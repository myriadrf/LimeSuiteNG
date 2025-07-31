@echo off

echo # - Starting LimeSuiteNG conda package installation
:: set dependencies
set CONDA_DEPS="vs2022_win-64 cmake ninja"

FOR /f "delims=" %%i in ('conda info ^| findstr /C:"active environment"') do set CURR_CONDA_ENV=%%i
IF ERRORLEVEL 1 (
   echo # - DEBUG: curr env - %CURR_CONDA_ENV%
   echo # - Error: Using invalid prompt. Use radioconda prompt with admin privileges!
   EXIT /B
)

set "CURR_CONDA_ENV=%CURR_CONDA_ENV:*active environment : =%"


IF "%CURR_CONDA_ENV%"=="base" (
   echo # - DEBUG: CURR_CONDA_ENV = %CURR_CONDA_ENV%
   echo # - Error: Trying to install conda packages into base environment!
   echo # - Activate your environment:
   echo # -
   echo # - conda activate ^<your custom env name^>
   echo # -
   EXIT /B
)

@REM FOR /f "delims=" %%i in ('conda list conda-build ^| findstr "conda-build"') do set CONDA_BUILD_PKG=%%i
@REM IF NOT %CONDA_BUILD_PKG%="conda-build"(
@REM    echo "# - DEBUG: CONDA_BUILD_PKG = %CONDA_BUILD_PKG%"
@REM    echo "# - conda-build tools package missing! Adding package to dependency list."
@REM    set "CONDA_DEPS=%CONDA_DEPS% conda-build"
@REM )
@REM ELSE (
@REM    echo "# - conda-build package detected. Skipping."
@REM )


@REM FOR /f "delims=" %%i in ('conda list conda-forge-pinning ^| findstr "conda-forge-pinning"') do set CONDA_FORGE_PKG=%%i
@REM IF NOT %CONDA_FORGE_PKG%="conda-forge-pinning" (
@REM    echo "# - DEBUG: CONDA_FORGE_PKG = %CONDA_FORGE_PKG%"
@REM    echo "# - conda-forge-pinning package missing! Adding package to dependency list."
@REM    set "CONDA_DEPS=%CONDA_DEPS% conda-forge-pinning"
@REM )
@REM ELSE (
@REM    echo "# - conda-forge-pinning package detected. Skipping."
@REM )

@REM echo "# - Installing following conda packages for LimeSuiteNG build"
@REM FOR /f "tokens=*" %%i in (%CONDA_DEPS%) do (
@REM    echo "# - %%i"
@REM )

::CALL "conda install %CONDA_DEPS%"


