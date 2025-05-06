@echo off
echo Compiling Missile Interceptor Prototype...

rem Setup Visual Studio environment
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
echo Visual Studio environment has been set up.

rem Compile the prototype
set SOURCES=interceptor_prototype.cpp matrix.cpp perceptron_layer.cpp activation.cpp neural_network.cpp loss.cpp optimizer.cpp game.cpp
set OUTPUT_NAME=interceptor_prototype.exe

echo Compiling %SOURCES%...
cl.exe /EHsc /std:c++17 %SOURCES% /Fe%OUTPUT_NAME%

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Compilation successful! Executable: %OUTPUT_NAME%
    echo.
    echo Running Missile Interceptor Prototype...
    echo.
    %OUTPUT_NAME%
) else (
    echo Compilation failed with error code %ERRORLEVEL%
    exit /b 1
) 