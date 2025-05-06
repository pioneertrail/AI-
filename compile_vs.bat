@echo off
REM Batch script to compile the Missile Interceptor Simulation using Visual Studio C++ compiler

REM Find vcvarsall.bat (adjust path if needed)
set VCVARS_PATH="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"

REM Check if vcvarsall.bat exists
if not exist %VCVARS_PATH% (
    echo Error: vcvarsall.bat not found at %VCVARS_PATH%
    echo Please adjust the VCVARS_PATH variable in this script.
    exit /b 1
)

REM Set up Visual Studio environment (use x64 for 64-bit build)
echo Setting up Visual Studio environment...
call %VCVARS_PATH% x64
if errorlevel 1 (
    echo Error setting up Visual Studio environment.
    exit /b 1
)
echo Visual Studio environment has been set up.

REM Compile the source files (Add console_renderer.cpp)
echo Compiling Missile Interceptor Simulation (src/missile_sim_main.cpp src/missile_interceptor.cpp src/console_renderer.cpp)...
cl /EHsc /std:c++17 /Zi /I src src\missile_sim_main.cpp src\missile_interceptor.cpp src\console_renderer.cpp /link /out:missile_sim.exe /DEBUG

REM Check compilation result
if errorlevel 1 (
    echo.
    echo Compilation failed with error code %errorlevel%
    exit /b %errorlevel%
) else (
    echo.
    echo Compilation successful! Executable: missile_sim.exe
)

REM Optional: Run the compiled executable
echo.
echo Running Missile Interceptor Simulation...
echo.
.\missile_sim.exe

if errorlevel 1 (
    echo Simulation failed with error code %errorlevel%
) else (
    echo.
    echo Missile simulation complete.
)