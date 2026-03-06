@echo off
cd /d E:\Github\ProjectAirSim
call "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44
set CMAKE_INSTALL_PREFIX=
build.cmd package_plugin
