@echo off
cd /d "%~dp0"
set "PY="

echo ==========================================
echo   STM32F407 Dev Environment Setup
echo ==========================================
echo.

:: ===============================================
:: Step 0: Find Python
:: ===============================================
for %%P in (python python3) do (
    where %%P >nul 2>&1
    if not errorlevel 1 (
        set "PY=%%P"
        goto :found_python
    )
)
for %%P in (
    "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
    "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
    "C:\Python\python.exe"
    "C:\Python311\python.exe"
    "C:\Python312\python.exe"
) do (
    if exist %%P (
        set "PY=%%P"
        goto :found_python
    )
)

echo [ERROR] Python not found. Install Python 3.11+ first.
echo         https://www.python.org/downloads/
pause
exit /b 1

:found_python
echo [*] Python: %PY%

:: ===============================================
:: Step 1: ARM GCC in C:\tools\ (shared)
:: ===============================================
echo [*] Checking ARM GCC...
if exist "C:\tools\arm-gcc\bin\arm-none-eabi-gdb.exe" (
    echo     Found in C:\tools\arm-gcc\ (shared)
    goto :link_gcc
)

echo     Not found in C:\tools. Looking for existing installation...

for %%P in (
    "C:\path\C_C++\14.2 rel1\bin\arm-none-eabi-gdb.exe"
) do (
    if exist %%P (
        echo     Copying from %%~dpP..
        xcopy "%%~dpP..\*" "C:\tools\arm-gcc\" /E /I /Q /H /Y >nul
        goto :link_gcc
    )
)

echo.
echo [WARN] ARM GCC not found!
echo.
echo Please download ARM GNU Toolchain:
echo   https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
echo   Choose: Windows mingw-w64-i686, arm-none-eabi, .zip
echo.
echo Extract to: C:\tools\arm-gcc\
echo (Ensure C:\tools\arm-gcc\bin\arm-none-eabi-gdb.exe exists)
echo Then run this script again.
echo.
pause
exit /b 1

:link_gcc
:: ===============================================
:: Step 2: Create junction in this project
:: ===============================================
if exist "%~dp0tools\arm-gcc" (
    echo [*] tools\arm-gcc already exists
) else (
    echo [*] Creating junction: tools\arm-gcc -^> C:\tools\arm-gcc
    mkdir "%~dp0tools" 2>nul
    powershell -Command "New-Item -Path '%~dp0tools\arm-gcc' -ItemType Junction -Target 'C:\tools\arm-gcc' -Force" >nul 2>&1
)

:: ===============================================
:: Step 3: Python venv in C:\tools\ (shared)
:: ===============================================
echo [*] Checking Python venv...
if exist "C:\tools\python_venv\Scripts\pyocd.exe" (
    echo     Found in C:\tools\python_venv\ (shared)
    goto :link_venv
)

echo     Creating shared venv at C:\tools\python_venv...
%PY% -m venv --copies "C:\tools\python_venv"
if errorlevel 1 (
    echo [ERROR] venv creation failed!
    pause
    exit /b 1
)

echo     Installing pyOCD...
call "C:\tools\python_venv\Scripts\python.exe" -m pip install --upgrade pip -q --disable-pip-version-check
call "C:\tools\python_venv\Scripts\python.exe" -m pip install pyocd -q --disable-pip-version-check
if errorlevel 1 (
    echo [ERROR] pyOCD install failed. Check network.
    pause
    exit /b 1
)
echo     pyOCD installed.

:link_venv
:: ===============================================
:: Step 4: Create junction in this project
:: ===============================================
if exist "%~dp0tools\python_venv" (
    echo [*] tools\python_venv already exists
) else (
    echo [*] Creating junction: tools\python_venv -^> C:\tools\python_venv
    mkdir "%~dp0tools" 2>nul
    powershell -Command "New-Item -Path '%~dp0tools\python_venv' -ItemType Junction -Target 'C:\tools\python_venv' -Force" >nul 2>&1
)

:: ===============================================
:: Step 5: CMSIS Pack
:: ===============================================
echo [*] Installing STM32F407 CMSIS Pack...
call "C:\tools\python_venv\Scripts\pyocd.exe" pack install stm32f407vetx
if errorlevel 1 (
    echo [WARN] Pack install failed. Run manually later:
    echo        C:\tools\python_venv\Scripts\pyocd.exe pack install stm32f407vetx
)

:: ===============================================
:: Done
:: ===============================================
echo.
echo ==========================================
echo   Setup Complete!
echo ==========================================
echo.
echo   Shared tools at C:\tools\
echo   Junctions created in %~dp0tools\
echo.
echo   Connect DAPLink, open VSCode, press F5.
echo ==========================================
pause
