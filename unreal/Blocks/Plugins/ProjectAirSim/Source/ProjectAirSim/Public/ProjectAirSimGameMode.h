// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "UObject/SoftObjectPtr.h"
#include "UnrealSimLoader.h"

//
#include "ProjectAirSimGameMode.generated.h"

class AController;
class APawn;
class UInputMappingContext;

UCLASS(Config = Game)
class PROJECTAIRSIM_API AProjectAirSimGameMode : public AGameModeBase {
  GENERATED_BODY()

 public:
  explicit AProjectAirSimGameMode(const FObjectInitializer& ObjectInitializer);

  UClass* GetDefaultPawnClassForController_Implementation(
      AController* InController) override;

  void StartPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  UPROPERTY(EditDefaultsOnly, Config, Category = "Pawn")
  bool bUseThirdPersonPawn = true;

  UPROPERTY(EditDefaultsOnly, Config, Category = "Pawn")
  TSoftClassPtr<APawn> ThirdPersonPawnClass;

  // When using a steppable clock, ProjectAirSim pauses the Unreal world
  // between sim steps. Enable this to keep Unreal running while the player
  // view is active so a third-person pawn can move.
  UPROPERTY(EditDefaultsOnly, Config, Category = "Pawn")
  bool bAllowPlayerMovementWithSteppableClock = false;

  // Optional input mapping context to apply when the player view is active.
  // This is useful for Enhanced Input setups (ex. ThirdPerson template).
  UPROPERTY(EditDefaultsOnly, Config, Category = "Pawn")
  TSoftObjectPtr<UInputMappingContext> ThirdPersonInputMappingContext;

  // Optional secondary mapping context (ex. mouse look when default is gamepad).
  UPROPERTY(EditDefaultsOnly, Config, Category = "Pawn")
  TSoftObjectPtr<UInputMappingContext> ThirdPersonInputMappingContextSecondary;

 private:
  AUnrealSimLoader UnrealSimLoader;
};
