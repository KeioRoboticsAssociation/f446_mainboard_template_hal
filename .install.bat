@echo off
setlocal enabledelayedexpansion

rem Check if the project name is provided as an argument
if "%~1"=="" (
    echo Usage: install.bat [project_name]
    exit /b 1
) else (
    set project_name=%~1
)

rem Change to the directory of the script
cd /d %~dp0

rem Rename the .ioc file and replace content
if exist f446_template_hal.ioc (
    move f446_template_hal.ioc %project_name%.ioc
    powershell -Command "(Get-Content -path %project_name%.ioc) -replace 'f446_template_hal', '%project_name%' | Set-Content -path %project_name%.ioc"
)

rem Replace content in CMakeLists.txt
if exist CMakeLists.txt (
    powershell -Command "(Get-Content -path CMakeLists.txt) -replace 'f446_template_hal', '%project_name%' | Set-Content -path CMakeLists.txt"
)

rem Delete the .install.sh script
if exist .install.sh (
    del /f .install.sh
)

rem Delete the install.bat script itself
del /f "%~f0"

rem Clear the content of README.md
if exist README.md (
    echo. > README.md
)

rem Delete the .git directory
if exist .git (
    rmdir /s /q .git
)

endlocal