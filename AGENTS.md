# ProjectAirSim handoff notes (UE 5.7 upgrade)

## Summary
- UE 5.7 source build works; Unreal Editor launches with `unreal\Blocks\Blocks.uproject`.
- Plugin packaging uses `build.cmd package_plugin`.
- New helper script added: `E:\Github\ProjectAirSim\build_ue57.cmd` (sets UE env vars, clears `CMAKE_INSTALL_PREFIX`, runs VS dev cmd, then packages plugin).
- `mt-80_ardupilot.jsonc` updated: radar/lidar debug points enabled, mesh path updated to plugin mount.

## Critical gotchas
- **Do NOT set `CMAKE_INSTALL_PREFIX` globally.** It breaks dependency installs (NNG/assimp) by trying to write to Program Files.
- If `build.cmd package_plugin` fails installing NNG/assimp to Program Files, delete cached deps and rebuild:
  - `build\win64\Debug\_deps\nng*`, `build\win64\Debug\_deps\assimp*`
  - `build\win64\Release\_deps\nng*`, `build\win64\Release\_deps\assimp*`
  - Then run `build_ue57.cmd` again.
- In PowerShell, user profile scripts are blocked; use `cmd.exe` or `powershell -NoProfile` to avoid `PSSecurityException`.

## UE 5.7 build env
- UE root: `E:\UE_5.7`
- Toolchain: MSVC 14.44.x
- Typical build cmd (used by `build_ue57.cmd`):
  - `call "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -vcvars_ver=14.44`
  - `build.cmd package_plugin`

## Plugin + mesh content
- New EFT_X2300 mesh moved into plugin content:
  - `unreal\Blocks\Plugins\Drone\Content\EFT_X2300.uasset`
  - Ensure materials are also in plugin content, not only under `unreal\Blocks\Content\`.
- Old copies still exist under project content:
  - `unreal\Blocks\Content\EFT_X2300*.uasset`
  - Recommended: fix redirectors in UE and remove old `/Game` copies to avoid fallback.

## Config updates made
- Mesh references updated from `/Game/EFT_X2300.EFT_X2300` to `/Drone/EFT_X2300.EFT_X2300`:
  - `client\python\example_user_scripts\ardupilot\sim_config\mt-80_ardupilot.jsonc`
  - `client\python\example_user_scripts\ardupilot\sim_config\robot_ardu_quadrotor.jsonc`
- Radar/Lidar visualization enabled:
  - In `mt-80_ardupilot.jsonc`, both `draw-debug-points` values set to `true` for `livox_lidar` and `Radar1`.

## Packaged plugin output
- Packaging outputs to: `packages\projectairsim_ue_plugin\Plugins\`
  - `ProjectAirSim`, `Drone`, `Rover`
- After packaging, verify mesh exists in packaged Drone plugin:
  - `packages\projectairsim_ue_plugin\Plugins\Drone\Content\EFT_X2300.uasset`

## Python test (optional)
- Existing script loads the mt-80 config:
  - `client\python\example_user_scripts\ardupilot\ardupilot_quadrotor.py`
  - It uses `scene_ardu_quadrotor.jsonc`, which references `mt-80_ardupilot.jsonc`.
- Example run:
  - `set PYTHONPATH=E:\Github\ProjectAirSim\client\python\projectairsim\src`
  - `C:\Users\Dogukan\AppData\Local\Programs\Python\Python310\python.exe client\python\example_user_scripts\ardupilot\ardupilot_quadrotor.py`

## Extra context (UE 5.7 fixes already done)
- UE 5.7 API updates applied across plugin code (RHI texture type changes, line batcher API, FRDG changes, logging format string changes, missing includes).
- If new compile errors appear, search recent diffs around these areas.
