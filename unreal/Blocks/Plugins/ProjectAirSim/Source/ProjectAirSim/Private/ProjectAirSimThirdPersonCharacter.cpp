// Copyright (C) Microsoft Corporation.
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#include "ProjectAirSimThirdPersonCharacter.h"

#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "UObject/ConstructorHelpers.h"

AProjectAirSimThirdPersonCharacter::AProjectAirSimThirdPersonCharacter() {
  GetCapsuleComponent()->InitCapsuleSize(42.0f, 96.0f);

  bUseControllerRotationPitch = false;
  bUseControllerRotationYaw = false;
  bUseControllerRotationRoll = false;

  UCharacterMovementComponent* Movement = GetCharacterMovement();
  Movement->bOrientRotationToMovement = true;
  Movement->RotationRate = FRotator(0.0f, 540.0f, 0.0f);
  Movement->JumpZVelocity = 600.0f;
  Movement->AirControl = 0.2f;

  CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
  CameraBoom->SetupAttachment(RootComponent);
  CameraBoom->TargetArmLength = 300.0f;
  CameraBoom->bUsePawnControlRotation = true;

  FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
  FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
  FollowCamera->bUsePawnControlRotation = false;

  BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
  BodyMesh->SetupAttachment(GetCapsuleComponent());
  BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  BodyMesh->SetRelativeLocation(FVector(0.0f, 0.0f, -90.0f));
  BodyMesh->SetRelativeScale3D(FVector(0.7f, 0.7f, 1.0f));

  static ConstructorHelpers::FObjectFinder<UStaticMesh> CapsuleMesh(
      TEXT("/Engine/BasicShapes/Capsule.Capsule"));
  if (CapsuleMesh.Succeeded()) {
    BodyMesh->SetStaticMesh(CapsuleMesh.Object);
  }
}

void AProjectAirSimThirdPersonCharacter::SetupPlayerInputComponent(
    UInputComponent* PlayerInputComponent) {
  check(PlayerInputComponent);
  Super::SetupPlayerInputComponent(PlayerInputComponent);

  PlayerInputComponent->BindAxis("MoveForward", this,
                                 &AProjectAirSimThirdPersonCharacter::MoveForward);
  PlayerInputComponent->BindAxis("MoveRight", this,
                                 &AProjectAirSimThirdPersonCharacter::MoveRight);
  PlayerInputComponent->BindAxis("Turn", this,
                                 &AProjectAirSimThirdPersonCharacter::Turn);
  PlayerInputComponent->BindAxis("LookUp", this,
                                 &AProjectAirSimThirdPersonCharacter::LookUp);
  PlayerInputComponent->BindAction("Jump", IE_Pressed, this,
                                   &ACharacter::Jump);
  PlayerInputComponent->BindAction("Jump", IE_Released, this,
                                   &ACharacter::StopJumping);
}

void AProjectAirSimThirdPersonCharacter::MoveForward(float Value) {
  if (!Controller || FMath::IsNearlyZero(Value)) return;

  const FRotator Rotation = Controller->GetControlRotation();
  const FRotator YawRotation(0.0f, Rotation.Yaw, 0.0f);
  const FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
  AddMovementInput(Direction, Value);
}

void AProjectAirSimThirdPersonCharacter::MoveRight(float Value) {
  if (!Controller || FMath::IsNearlyZero(Value)) return;

  const FRotator Rotation = Controller->GetControlRotation();
  const FRotator YawRotation(0.0f, Rotation.Yaw, 0.0f);
  const FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
  AddMovementInput(Direction, Value);
}

void AProjectAirSimThirdPersonCharacter::Turn(float Value) {
  AddControllerYawInput(Value);
}

void AProjectAirSimThirdPersonCharacter::LookUp(float Value) {
  AddControllerPitchInput(Value);
}
