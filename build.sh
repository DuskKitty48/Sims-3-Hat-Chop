#!/bin/sh
# Builds bin/HatChop.asi (32-bit). Run in the MSYS2 "MINGW32" shell (pacman -S mingw-w64-i686-gcc).
set -e
cd "$(dirname "$0")"
mkdir -p bin
gcc -O2 -Wall -Wextra -Wno-unused-function -Wno-cast-function-type -Wno-int-to-pointer-cast \
    -Wno-pointer-to-int-cast -static-libgcc -msse2 -mfpmath=sse \
    -shared -s -o bin/HatChop.asi src/hat_chop_hat_fit.c -lm
echo "Built bin/HatChop.asi"
