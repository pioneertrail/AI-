@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
echo Visual Studio environment has been set up.

REM Compile the main project source files
set SOURCES=main.cpp matrix.cpp
set OUTPUT_NAME=ml_app.exe

echo Compiling %SOURCES%...
cl.exe /EHsc /std:c++17 %SOURCES% /Fe%OUTPUT_NAME%

echo.
if %ERRORLEVEL% EQU 0 (
    echo Compilation successful! Executable: %OUTPUT_NAME%
    echo Running program:
    echo.
    %OUTPUT_NAME%
) else (
    echo Compilation failed with error code %ERRORLEVEL%
) 