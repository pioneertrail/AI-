@echo off
echo Compiling Missile Interceptor Test...

rem Setup Visual Studio environment
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
echo Visual Studio environment has been set up.

rem Compile the test
cl.exe /EHsc /std:c++17 interceptor_test.cpp /Feinterceptor_test.exe

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Compilation successful! Executable: interceptor_test.exe
    echo.
    echo Running Missile Interceptor Test...
    echo.
    interceptor_test.exe
) else (
    echo Compilation failed with error code %ERRORLEVEL%
    exit /b 1
) 