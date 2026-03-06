// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "UObject/SoftObjectPtr.h"
#include "Renderers/BlackSharkRenderer.hpp"
#include "Robot/UnrealEnvActor.h"
#include "Robot/UnrealRobot.h"
#include "Sensors/UnrealViewportCamera.h"
#include "World/TimeofDay.hpp"
#include "World/WorldSimApi.h"
#include "core_sim/clock.hpp"
#include "core_sim/earth_utils.hpp"
#include "core_sim/scene.hpp"
#include "json.hpp"
#include "unreal_physics.hpp"

// comment so that generated.h is always the last include file with clang-format
#include "UnrealScene.generated.h"

class AActor;
class APlayerController;
class UInputMappingContext;

UCLASS()
class AUnrealScene : public AActor {
  GENERATED_BODY()

 public:
  explicit AUnrealScene(const FObjectInitializer& ObjectInitialize);

  void LoadUnrealScene(
      UWorld* World, microsoft::projectairsim::Scene& Scene,
      const std::unordered_map<std::string,
                               microsoft::projectairsim::UnrealPhysicsBody*>&
          UnrealPhysicsBodies);

  void UnloadUnrealScene();

  void SwitchStreamingView();

  void TogglePlayerDroneView();

  void ToggleTrace();

  void SetTraceLine(const std::vector<float>& color_rgba, float thickness);

  void StartUnrealScene();

  void StopUnrealScene();

  void Tick(float DeltaTime) override;

  bool GetUnrealBoundingBox3D(
      const std::string& ObjectName,
      microsoft::projectairsim::BoxAlignment BoxAlignment,
      FOrientedBox& OutOrientedBox, FRotator& OutRotation) const;

  bool GetSimBoundingBox3D(const std::string& object_name,
                           microsoft::projectairsim::BoxAlignment box_alignment,
                           FOrientedBox& OutUnrealBox,
                           microsoft::projectairsim::BBox3D& OutSimBox) const;

  AActor* FindActor(const std::string& object_name) const;

  TMap<FString, AActor*> scene_object_map;
  TMap<FString, FAssetData> asset_map_;

 protected:
  void BeginPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

 private:
  void LoadUnrealActor(
      UWorld* World, const microsoft::projectairsim::Actor& Actor,
      const std::unordered_map<std::string,
                               microsoft::projectairsim::UnrealPhysicsBody*>&
          UnrealPhysicsBodies);

  void LoadUnrealEnvActor(UWorld* World,
                          const microsoft::projectairsim::Actor& Actor);

  bool GetWorldAlignedBoundingBox3D(const std::string& ObjectName,
                                    FOrientedBox& OrientedBox) const;
  bool GetActorAlignedBoundingBox3D(const std::string& ObjectName,
                                    FOrientedBox& OrientedBox,
                                    FRotator& OutRotation) const;

  void RegisterServiceMethods();

  bool SwitchStreamingViewServiceMethod();

  bool SetTraceLineServiceMethod(const std::vector<float>& color_rgba,
                                 float thickness);

  bool ToggleTraceServiceMethod();

  void UpdateWindVelocity(const microsoft::projectairsim::Vector3& wind_vel);

  void EnableUnrealViewportCamera(bool enable);
  void EnsurePlayerInputEnabled(APlayerController* PlayerController);
  void ApplyThirdPersonInputMapping(APlayerController* PlayerController);
  void ApplyInputMappingContext(APlayerController* PlayerController,
                                const TSoftObjectPtr<UInputMappingContext>&
                                    MappingContextRef);

  nlohmann::json Get3DBoundingBoxServiceMethod(const std::string& object_name,
                                               int box_alignment);

  AUnrealRobot* GetViewTargetRobot();
  bool SetViewTargetToRobot(APlayerController* PlayerController);

  UWorld* unreal_world;
  microsoft::projectairsim::Scene* sim_scene;
  microsoft::projectairsim::HomeGeoPoint home_geo_point;
  AUnrealViewportCamera* unreal_viewport_camera_;
  TArray<AUnrealRobot*> unreal_actors;
  TArray<AUnrealEnvActor*> unreal_env_actors;
  size_t idx_actor_to_view = 0;
  bool found_actor = false;
  bool is_player_view_active_ = false;
  TWeakObjectPtr<AActor> player_view_target_;
  bool prefer_player_view_ = false;
  bool pending_player_view_init_ = false;
  bool allow_player_unpaused_ = false;
  bool warned_missing_input_context_ = false;
  TSoftObjectPtr<UInputMappingContext> third_person_input_context_;
  TSoftObjectPtr<UInputMappingContext> third_person_input_context_secondary_;

  FCriticalSection UpdateMutex;

  TimeNano unreal_time;
  bool using_unreal_physics;

  std::unique_ptr<WorldSimApi> world_api;
  std::shared_ptr<TimeOfDay> time_of_day;
  std::vector<std::string> objects_;

  UClass* sky_sphere_class_;
  TimeOfDaySetting tod_setting;
  std::shared_ptr<BlackSharkRenderer>  black_shark_renderer;

  // actor name -> scaled bbox in actor space
  mutable std::unordered_map<std::string, FBox> actor_bbox_cache;

  // trace path variables
  bool tracing_enabled = false;
  FColor trace_color = FColor::Purple;
  float trace_thickness = 3.0f;

  FVector last_position;
};
