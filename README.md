# CPU-R4300i-cpp

## NINTENDO64 Emulator

### WIP

<br><br><br>

### requires a PIF image (pifdata.bin)

<br><br><br>

Ubuntu 22.04

<pre>
sudo apt install build-essential cmake clang-format mesa-common-dev libopenal-dev
</pre>

<br><br><br>

<br>


https://user-images.githubusercontent.com/10168979/190008980-cfb05b37-bd01-4a24-ba6c-a4d48ed8d1e7.mp4




use cen64 as a reference

<br><br><br><br><br>

## Technical information

<br>

| ROM format | Type          | First 4 bytes | Game Title in ROM |
| ---------- | ------------- | ------------- | ----------------- |
| .z64       | Big Endian    | 80 37 12 40   | `SUPER MARIO 64 ` |
| .v64       | Byteswapped   | 37 80 40 12   | `USEP RAMIR O64 ` |
| .n64       | Little Endian | 40 12 37 80   | `EPUSAM R OIR 46` |

<br><br><br><br><br><br>

vscode extensions

<pre>
    C/C++
    C/C++ Extension Pack
    Better C++ Syntax
    CMake
    CMake Tools
    CodeLLDB
    Makefile Tools
    IntelliCode
    clangd
</pre>

<br><br><br>

(Ctrl + Shift + p)  
CMake: Configure

<br><br><br>

### F7

Build

<br>

### F5

debug

<br><br><br><br><br><br><br><br><br><br>
