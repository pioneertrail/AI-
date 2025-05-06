@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
echo Visual Studio environment has been set up.

REM Define source files and output name for Missile Simulation
set SOURCES=src/missile_sim_main.cpp src/missile_interceptor.cpp
set OUTPUT_NAME=missile_sim.exe

echo Compiling Missile Interceptor Simulation (%SOURCES%)...
cl.exe /EHsc /std:c++17 /W4 /Zi %SOURCES% /Fe%OUTPUT_NAME%

echo.
if %ERRORLEVEL% EQU 0 (
    echo Compilation successful! Executable: %OUTPUT_NAME%
    echo.
    echo Running Missile Interceptor Simulation...
    echo.
    %OUTPUT_NAME%
    echo.
    echo Missile simulation complete.
) else (
    echo Compilation failed with error code %ERRORLEVEL%
)