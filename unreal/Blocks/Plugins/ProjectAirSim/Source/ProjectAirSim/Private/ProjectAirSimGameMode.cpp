// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#include "ProjectAirSimGameMode.h"

#include <exception>

#include "Runtime/Core/Public/Misc/Paths.h"

AProjectAirSimGameMode::AProjectAirSimGameMode(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer),
      UnrealSimLoader(FPaths::ConvertRelativePathToFull(FPaths::ProjectDir())) {
  FApp::bUseFixedSeed = true;  // for determinism, persists in UE project
}

UClass* AProjectAirSimGameMode::GetDefaultPawnClassForController_Implementation(
    AController* InController) {
  if (bUseThirdPersonPawn && !ThirdPersonPawnClass.IsNull()) {
    if (UClass* LoadedClass = ThirdPersonPawnClass.LoadSynchronous()) {
      return LoadedClass;
    }
  }

  return Super::GetDefaultPawnClassForController_Implementation(InController);
}

void AProjectAirSimGameMode::StartPlay() {
  Super::StartPlay();

  UnrealSimLoader.LaunchSimulation(this->GetWorld());
}

void AProjectAirSimGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  Super::EndPlay(EndPlayReason);

  UnrealSimLoader.TeardownSimulation();
  FApp::bUseFixedSeed = false;  // reset back to default from App.cpp
}
