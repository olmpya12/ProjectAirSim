// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#include "ProjectAirSim.h"

#include "GlobalShader.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

#define LOCTEXT_NAMESPACE "FProjectAirSimModule"

void FProjectAirSimModule::StartupModule() {
  // This code will execute after your module is loaded into memory; the exact
  // timing is specified in the .uplugin file per-module
  const TSharedPtr<IPlugin> Plugin =
      IPluginManager::Get().FindPlugin(TEXT("ProjectAirSim"));
  if (!Plugin.IsValid()) {
    UE_LOG(LogTemp, Warning,
           TEXT("ProjectAirSim plugin not found; skipping shader mapping."));
    return;
  }

  const FString ShaderDirectory = FPaths::Combine(
      Plugin->GetBaseDir(), TEXT("Source/ProjectAirSim/Private/Shaders"));
  if (!FPaths::DirectoryExists(ShaderDirectory)) {
    UE_LOG(LogTemp, Warning,
           TEXT("ProjectAirSim shader directory not found: %s"),
           *ShaderDirectory);
    return;
  }

  AddShaderSourceDirectoryMapping(TEXT("/CustomShaders"), ShaderDirectory);
}

void FProjectAirSimModule::ShutdownModule() {
  // This function may be called during shutdown to clean up your module.  For
  // modules that support dynamic reloading, we call this function before
  // unloading the module.
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FProjectAirSimModule, ProjectAirSim)
