@echo off
setlocal
set ROOT_DIR=%CD%

chdir /d %ROOT_DIR% 
git submodule update --init --recursive

WHERE cmake >nul 2>nul
IF %ERRORLEVEL% NEQ 0 (
	call :installcmake
)

IF NOT EXIST external\rpclib\build mkdir external\rpclib\build
cd external\rpclib\build
cmake -G"Visual Studio 14 2015 Win64" ..
cmake --build .
cmake --build . --config Release
if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

set RPCLIB_TARGET_LIB=AirLib\deps\rpclib\lib\x64
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%

set RPCLIB_TARGET_INCLUDE=AirLib\deps\rpclib\include
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%

robocopy /MIR external\rpclib\include %RPCLIB_TARGET_INCLUDE%
robocopy /MIR external\rpclib\build\output\lib %RPCLIB_TARGET_LIB%


msbuild /p:Platform=x64 /p:Configuration=Debug AirSim.sln
if ERRORLEVEL 1 goto :buildfailed
msbuild /p:Platform=x64 /p:Configuration=Release AirSim.sln 
if ERRORLEVEL 1 goto :buildfailed

set MAVLINK_TARGET_LIB=AirLib\deps\MavLinkCom\lib
if NOT exist %MAVLINK_TARGET_LIB% mkdir %MAVLINK_TARGET_LIB%

set MAVLINK_TARGET_INCLUDE=AirLib\deps\MavLinkCom\include
if NOT exist %MAVLINK_TARGET_INCLUDE% mkdir %MAVLINK_TARGET_INCLUDE%

if NOT exist Unreal\Plugins\AirSim\Source\AirLib mkdir Unreal\Plugins\AirSim\Source\AirLib

robocopy /MIR MavLinkCom\include %MAVLINK_TARGET_INCLUDE%
robocopy /MIR MavLinkCom\lib %MAVLINK_TARGET_LIB%
robocopy /MIR AirLib Unreal\Plugins\AirSim\Source\AirLib  /XD temp
goto :eof

:buildfailed
chdir /d %ROOT_DIR% 
echo #### Build failed
goto :eof

:installcmake
if NOT EXIST cmake-3.7.2-win64-x64 call :downloadcmake
set PATH=%PATH%;%ROOT_DIR%\cmake-3.7.2-win64-x64\bin;
goto :eof

:downloadcmake
echo CMake was not found, so we are installing it for you... 
%ROOT_DIR%\tools\httpget "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
echo Decompressing cmake-3.7.2-win64-x64.zip...
%ROOT_DIR%\tools\unzip "cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
del cmake-3.7.2-win64-x64.zip
goto :eof

:cmakefailed
echo CMake install failed, please install cmake manually from https://cmake.org/
exit 1