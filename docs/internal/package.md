# Packaging Your Project on Windows via Terminal

This guide explains how to package a Project AirSim-based Unreal Engine project entirely from the command line on Windows. It assumes Unreal Engine was installed via the Epic Games Launcher and the project structure follows the standard layout.

## Prerequisites

Before packaging, make sure you have:

* Installed Unreal Engine 5.2 via the [Epic Games Launcher](https://www.unrealengine.com/).
* Built the `SimLibs` in Release mode using the provided scripts.
* Verified that all required content and dependencies are correctly referenced.

## Step-by-Step Instructions

### 1. Set Up Your Environment

Open a terminal **as Administrator** and define the following environment variables (adjust paths as needed):

```bat
:: Set Unreal Engine root directory
set UE_PATH="C:\Program Files\Epic Games\UE_5.2\Engine"

:: Set project root directory
set PROJECT_PATH=C:\Path\To\Your\ProjectAirsim\unreal\Blocks

:: Define where to output the packaged build inside the repo
set OUTPUT_DIR=%PROJECT_PATH%\..\..\packages

:: Choose build configuration
set CONFIGURATION=Development
```

To validate the setup:

```bat
dir %UE_PATH%\Build\BatchFiles\RunUAT.bat

dir %PROJECT_PATH%\Blocks.uproject
```

### 2. Compile SimLibs (if not already built)

Navigate to the root of the repository and run:

```bat
cd C:\Path\To\Your\ProjectAirsim
build.cmd simlibs_release
```

This builds all required static and dynamic libraries in `Release` mode.

### 3. Run the Packaging Command

Use Unreal’s `RunUAT.bat` tool to package the project:

```bat
%UE_PATH%\Build\BatchFiles\RunUAT.bat BuildCookRun ^
 -project="%PROJECT_PATH%\Blocks.uproject" ^
 -noP4 -platform=Win64 -clientconfig=%CONFIGURATION% ^
 -cook -allmaps -build -stage -pak -archive ^
 -archivedirectory="%OUTPUT_DIR%"
```

This command:

* Cooks all project content.
* Builds binaries.
* Packs content into `.pak` files.
* Archives everything to the `packages` directory inside the root of Project AirSim.

### 4. Verify the Output

Once complete, verify that `%OUTPUT_DIR%\Windows\Blocks\` contains:

* `Blocks.exe` — the main executable
* `Binaries\Win64\` — required DLLs
* `Content\` — cooked assets
* `Plugins\ProjectAirSim\...` — plugin resources if included via RuntimeDependencies

To list contents:

```bat
tree /F %OUTPUT_DIR%\Windows\Blocks
```
---

© IAMAI Simulations — All rights reserved.
