// Copyright (C) Microsoft Corporation.
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"

#include "ProjectAirSimThirdPersonCharacter.generated.h"

UCLASS()
class PROJECTAIRSIM_API AProjectAirSimThirdPersonCharacter : public ACharacter {
  GENERATED_BODY()

 public:
  AProjectAirSimThirdPersonCharacter();

  void SetupPlayerInputComponent(
      class UInputComponent* PlayerInputComponent) override;

 private:
  void MoveForward(float Value);
  void MoveRight(float Value);
  void Turn(float Value);
  void LookUp(float Value);

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera",
            meta = (AllowPrivateAccess = "true"))
  class USpringArmComponent* CameraBoom;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera",
            meta = (AllowPrivateAccess = "true"))
  class UCameraComponent* FollowCamera;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Visual",
            meta = (AllowPrivateAccess = "true"))
  class UStaticMeshComponent* BodyMesh;
};
