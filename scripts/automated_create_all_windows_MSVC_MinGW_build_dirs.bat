@echo off
REM =========================================================
REM   Automated generation of all the dirs for win builds
REM    for x32/amd64, with/without Kinect support, and 
REM    for MSVC9, MSVC10 and MinGW
REM   To launch the build themselves and build packages, 
REM    see "automated_build_msvc_binary_package.bat"
REM 
REM  Copy this script to "d:\code" (in my laptop!), adjust 
REM   all the paths below and execute. 
REM 
REM                              Jose Luis Blanco, 2011-12
REM =========================================================

REM  === THIS IS WHERE MRPT SOURCE TREE IS FROM THE CWD ===
REM set MRPT_BASE_DIR=mrpt-0.9.5
set MRPT_BASE_DIR=mrpt-svn

REM =================== SET ALL IMPORTANT PATHS ===================

set msvc9_DIR=d:\Program Files (x86)\Microsoft Visual Studio 9.0
set msvc10_DIR=d:\Program Files (x86)\Microsoft Visual Studio 10.0
set CMAKE_DIR=d:\Program Files (x86)\CMake 2.8\bin
set LIBUSBDIR=d:\code\libusb-win32-bin
REM === Don't use backslash (\) in this:
REM Note: Use Mingw64 path even for both 32 and 64bit builds:
set MINGW_ROOT=D:/MinGW64
REM === wxWidgets directory base name will be: %WX_ROOT%-win%ARCHN%-%COMP%
set WX_ROOT=D:/code/wxWidgets-2.9.2

REM ==============================================================

REM msvc9 ========================
set COMP=msvc9
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen


set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen

REM msvc10 ========================
set COMP=msvc10
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen

set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen

:MINGW_PARTS
REM MinGW ========================
set COMP=mingw
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen

set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen


goto End



REM ===== Subroutine: Generate project dir ============
:subGen

set ARCH=x%ARCHN%
set DIR=%MRPT_BASE_DIR%-%COMP%-%ARCH%
if %KINECT%==1 set DIR=%DIR%-kinect
if %ARCHN%==32 set ARCH_NAME=x86
if %ARCHN%==64 set ARCH_NAME=amd64

set WXDIR=%WX_ROOT%-win%ARCHN%-%COMP%

if %COMP%==mingw GOTO :subGen_mingw
REM Visual Studio --------------------------
if %COMP%==msvc9 set MSVC_DIR=%msvc9_DIR%
if %COMP%==msvc10 set MSVC_DIR=%msvc10_DIR%
if %COMP%==msvc9 set CMAKE_GEN=Visual Studio 9 2008
if %COMP%==msvc10 set CMAKE_GEN=Visual Studio 10
if %ARCHN%==64 set CMAKE_GEN=%CMAKE_GEN% Win64

set FFMPEGDIR=D:/code/ffmpeg-win%ARCHN%-dev
if %ARCHN%==32 set WXLIBDIR=%WXDIR%/lib/vc_dll
if %ARCHN%==64 set WXLIBDIR=%WXDIR%/lib/vc_amd64_dll

GOTO :subGen_common

REM MinGw (32 or 64) -----------------------

:subGen_mingw
set CMAKE_GEN=MinGW Makefiles
set CMAKE_EXTRA1=-DCMAKE_C_COMPILER=%MINGW_ROOT%/bin/gcc.exe
set CMAKE_EXTRA2=-DCMAKE_CXX_COMPILER=%MINGW_ROOT%/bin/g++.exe
if %ARCHN%==32 set CMAKE_EXTRA3=-DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32 
if %ARCHN%==64 set CMAKE_EXTRA3=

set FFMPEGDIR=D:/code/ffmpeg-win%ARCHN%-dev
set WXLIBDIR=%WXDIR%/lib/gcc_dll

REM Common part to all compilers -----------
:subGen_common

mkdir %DIR%
cd %DIR%
set ALL_PARAMS=-DDISABLE_SWISSRANGER_3DCAM_LIBS=ON -DOpenCV_DIR=d:/code/opencv-%COMP%-%ARCH% -DMRPT_HAS_FFMPEG_WIN32=ON -DFFMPEG_WIN32_ROOT_DIR=%FFMPEGDIR% -DwxWidgets_ROOT_DIR=%WXDIR% -DwxWidgets_LIB_DIR=%WXLIBDIR%

if %ARCHN%==32 set LIBUSBLIB=%LIBUSBDIR%\lib\msvc\libusb.lib 
if %ARCHN%==64 set LIBUSBLIB=%LIBUSBDIR%\lib\msvc_x64\libusb.lib 

if %KINECT%==1 set ALL_PARAMS=%ALL_PARAMS% -DBUILD_KINECT=ON -DBUILD_KINECT_USE_FREENECT=ON -DLIBUSB_1_INCLUDE_DIR=%LIBUSBDIR%/include -DLIBUSB_1_LIBRARY=%LIBUSBLIB%

REM Create Project:
echo on
cmake ../%MRPT_BASE_DIR% -G "%CMAKE_GEN%" %ALL_PARAMS% %CMAKE_EXTRA1% %CMAKE_EXTRA2% %CMAKE_EXTRA3%

REM and insist to make sure wxWidgets and other vars have been fixed:
cmake . %ALL_PARAMS%
echo off

REM ---------------- Create compilation script ----------------
set PATH_FIL=paths_%COMP%_%ARCH_NAME%
if %KINECT%==1 set PATH_FIL=%PATH_FIL%-kinect
set PATH_FIL=%PATH_FIL%.bat

if NOT %COMP%==mingw set EXTRA_MINGW_PATHS=
if %COMP%==mingw set EXTRA_MINGW_PATHS=%MINGW_ROOT%/bin

echo SET PATH=C:\Windows\system32;C:\Windows;C:\Program Files\TortoiseSVN\bin;D:\code\opencv-%COMP%-%ARCH%\bin\Release;D:\code\opencv-%COMP%-%ARCH%\bin\Debug;%WXLIBDIR%;%FFMPEGDIR%/bin;%LIBUSBDIR%\bin\%ARCH_NAME%;%CMAKE_DIR%;%CD%\bin\Release;%CD%\bin\Debug%EXTRA_MINGW_PATHS% > %PATH_FIL%
if NOT %COMP%==mingw echo call "%MSVC_DIR%\VC\vcvarsall.bat" %ARCH_NAME% >> %PATH_FIL%

echo call %PATH_FIL% > AUTOBUILD.bat
echo call ..\%MRPT_BASE_DIR%\scripts\automated_build_msvc_binary_package.bat ..\%MRPT_BASE_DIR%\ >> AUTOBUILD.bat
echo move mrpt*.exe ..\%DIR%.exe >> AUTOBUILD.bat
echo rmdir /Q /S _CPack_Packages  >> AUTOBUILD.bat

cd ..

REM End of Subroutine
GOTO :EOF


REM =========== The END =========== 
:End