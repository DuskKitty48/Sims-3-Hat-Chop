@echo off
rem Builds bin\HatChop.asi (32-bit) with MSYS2 MinGW-w64 i686 gcc.
rem Install MSYS2, then in an MSYS2 terminal: pacman -S mingw-w64-i686-gcc
rem Set GCC to another i686 gcc.exe if yours is not in C:\msys64\mingw32\bin.
setlocal
if "%GCC%"=="" set "GCC=C:\msys64\mingw32\bin\gcc.exe"
for %%I in ("%GCC%") do set "PATH=%%~dpI;%PATH%"
cd /d "%~dp0"
if not exist bin mkdir bin
"%GCC%" -O2 -Wall -Wextra -Wno-unused-function -Wno-cast-function-type -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast -static-libgcc -msse2 -mfpmath=sse -shared -s -o bin\HatChop.asi src\hat_chop_hat_fit.c -lm
if errorlevel 1 exit /b 1
echo Built bin\HatChop.asi
