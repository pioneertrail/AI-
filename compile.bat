@echo off
REM Simple compile script for the project

echo Compiling project...

REM Using g++ compiler, C++17 standard, include debug symbols (-g)
g++ -std=c++17 -g main.cpp matrix.cpp -o ml_app.exe

if %errorlevel% neq 0 (
    echo Compilation failed!
    exit /b %errorlevel%
) else (
    echo Compilation successful! Executable: ml_app.exe
) 