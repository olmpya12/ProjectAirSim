// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.
// Unreal Radar sensor implementation

#pragma once

#include <deque>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "CoreMinimal.h"
#include "UnrealSensor.h"
#include "core_sim/clock.hpp"
#include "core_sim/message/radar_detection_message.hpp"
#include "core_sim/message/radar_track_message.hpp"
#include "core_sim/physics_common_types.hpp"
#include "core_sim/sensors/radar.hpp"

// comment so that generated.h is always the last include file with clang-format
#include "UnrealRadar.generated.h"

UCLASS() class UUnrealRadar : public UUnrealSensor {
  GENERATED_BODY()

 public:
  explicit UUnrealRadar(const FObjectInitializer& ObjectInitializer);

  void Initialize(const microsoft::projectairsim::Radar& SimRadar);

  void TickComponent(float DeltaTime, ELevelTick TickType,
                     FActorComponentTickFunction* ThisTickFunction) override;

 protected:
  void BeginPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

 private:
  using RadarBeamPoint = microsoft::projectairsim::RadarBeamPoint;
  using RadarDetection = microsoft::projectairsim::RadarDetection;

  void InitializePose(const microsoft::projectairsim::Transform& Pose);

  void GenerateFullFOVFrame();

  std::vector<RadarBeamPoint> GenerateBeamsToShoot(TimeNano SimTime);

  std::vector<RadarBeamPoint> GenerateContinuousBeams(int points_per_frame);

  FHitResult ShootSingleBeam(const FVector& RadarBodyLoc,
                             const FRotator& RadarBodyRot,
                             const RadarBeamPoint& BeamPoint);

  void SimulateRadarDetections(const TimeNano SimTime);

  void SimulateRadarTracks(const TimeNano SimTime);

  float EstimateRadarCrossSection(const AActor* Actor);

  microsoft::projectairsim::Vector3 ConvertToSensorFrame(
      const microsoft::projectairsim::Vector3& VecNED);

  microsoft::projectairsim::Kinematics GetKinematicsFromActor(const AActor* Actor);

  float SampleNormal(float stddev);

  float SampleUniform(float min_val, float max_val);

  float QuantizeValue(float value, float resolution, float origin = 0.0f);

  void FlushPendingMessages(TimeNano SimTime);

  void QueueDetectionMessage(TimeNano SimTime,
                             const microsoft::projectairsim::Pose& RadarPose,
                             const std::vector<RadarDetection>& Detections);

  void QueueTrackMessage(TimeNano SimTime,
                         const microsoft::projectairsim::Pose& RadarPose,
                         const std::vector<microsoft::projectairsim::RadarTrack>& Tracks);

  microsoft::projectairsim::Radar Radar;
  microsoft::projectairsim::RadarSettings Settings;
  AActor* OwnerActor;
  TimeNano DetectionInterval;
  TimeNano TrackInterval;
  TimeNano DataLatency;
  TimeNano LastDetectionTime;
  TimeNano LastTrackTime;
  int NextTrackID;
  std::vector<RadarBeamPoint> FullFOVFrame;
  TArray<FHitResult> AccumulatedGroundTruthHits;
  std::vector<RadarDetection> AccumulatedDetections;

  struct PendingDetectionMsg {
    TimeNano publish_time = 0;
    microsoft::projectairsim::RadarDetectionMessage message;

    PendingDetectionMsg(
        TimeNano time,
        const microsoft::projectairsim::RadarDetectionMessage& msg)
        : publish_time(time), message(msg) {}
  };

  struct PendingTrackMsg {
    TimeNano publish_time = 0;
    microsoft::projectairsim::RadarTrackMessage message;

    PendingTrackMsg(TimeNano time,
                    const microsoft::projectairsim::RadarTrackMessage& msg)
        : publish_time(time), message(msg) {}
  };

  std::deque<PendingDetectionMsg> PendingDetectionMsgs;
  std::deque<PendingTrackMsg> PendingTrackMsgs;
  std::mt19937 Rng;

  // Track calculation data
  std::unordered_map<int, std::string> TrackIDToName;
  std::unordered_map<std::string, int> TrackNameToID;
  std::unordered_map<int, microsoft::projectairsim::RadarTrack> TrackIDToTrack;
  std::unordered_map<int, int> TrackIDToAssocCnt;
  std::unordered_map<int, bool> TrackIDToConfirmation;

  static constexpr int kTrackConfirmThresh = 3;
  static constexpr int kTrackNumLastDetections = 5;
  static constexpr int kTrackDeleteThresh = 3;
};
