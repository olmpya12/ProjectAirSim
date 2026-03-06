@echo off
setlocal

REM Build ProjectAirSim plugin for UE 5.7 with expected toolchain.
set "UE_ROOT=E:\UE_5.7"
set "UE_COMPILER_VERSION=14.44.35207"
set "UE_TOOLCHAIN_VERSION=14.44.35207"
set "CMAKE_INSTALL_PREFIX="


call build.cmd package_plugin
set EXIT_CODE=%errorlevel%
popd

exit /b %EXIT_CODE%
