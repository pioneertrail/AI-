@echo off
echo Compiling Missile Interceptor Simulation...
call compile_vs.bat
if errorlevel 1 (
    echo Compilation failed!
    exit /b 1
)

echo.
echo Running Missile Interceptor Simulation...
echo.
ml_app.exe %* 