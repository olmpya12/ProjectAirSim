// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.
// Unreal Radar sensor implementation

#include "UnrealRadar.h"

#include <algorithm>
#include <cmath>

#include "CollisionShape.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMeshActor.h"
#include "Robot/UnrealEnvActor.h"
#include "Robot/UnrealRobot.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "UnrealLogger.h"
#include "UnrealTransforms.h"
#include "core_sim/message/radar_detection_message.hpp"
#include "core_sim/message/radar_track_message.hpp"
#include "core_sim/transforms/transform_utils.hpp"

namespace projectairsim = microsoft::projectairsim;

using RadarBeamPoints = std::vector<projectairsim::RadarBeamPoint>;
using RadarDetections = std::vector<projectairsim::RadarDetection>;
using RadarTracks = std::vector<projectairsim::RadarTrack>;

UUnrealRadar::UUnrealRadar(const FObjectInitializer& ObjectInitializer)
    : UUnrealSensor(ObjectInitializer),
      DetectionInterval(0),
      TrackInterval(0),
      DataLatency(0),
      LastDetectionTime(0),
      LastTrackTime(0),
      NextTrackID(0) {
  bAutoActivate = true;

  // Tick in PostUpdateWork to update at the same time as UnrealCamera, instead
  // of waiting for the PrePhysics tick on the next loop
  PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
  PrimaryComponentTick.bCanEverTick = true;
  PrimaryComponentTick.bStartWithTickEnabled = true;

  std::random_device rd;
  Rng.seed(rd());
}

void UUnrealRadar::Initialize(const projectairsim::Radar& SimRadar) {
  Radar = SimRadar;
  Settings = SimRadar.GetRadarSettings();
  OwnerActor = GetOwner();

  DetectionInterval =
      projectairsim::SimClock::Get()->SecToNanos(Settings.detection_interval);
  TrackInterval =
      projectairsim::SimClock::Get()->SecToNanos(Settings.track_interval);
  DataLatency = Settings.data_latency <= 0.0f
                    ? 0
                    : projectairsim::SimClock::Get()->SecToNanos(
                          Settings.data_latency);

  AccumulatedGroundTruthHits.Empty();
  AccumulatedDetections.clear();

  TrackIDToName.clear();
  TrackNameToID.clear();
  TrackIDToTrack.clear();
  TrackIDToAssocCnt.clear();
  TrackIDToConfirmation.clear();
  PendingDetectionMsgs.clear();
  PendingTrackMsgs.clear();

  InitializePose(Settings.origin_setting);
  GenerateFullFOVFrame();
  RegisterComponent();
}

void UUnrealRadar::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

  // Use sim clock time instead of Unreal DeltaTime
  const TimeNano CurSimTime = projectairsim::SimClock::Get()->NowSimNanos();
  FlushPendingMessages(CurSimTime);
  const TimeNano DetectionElapsedTime = CurSimTime - LastDetectionTime;

  if (DetectionElapsedTime >= DetectionInterval) {
    // Simulate RADAR detections
    SimulateRadarDetections(CurSimTime);
    LastDetectionTime = CurSimTime;

    const TimeNano TrackElapsedTime = CurSimTime - LastTrackTime;
    if (TrackElapsedTime >= TrackInterval) {
      // Simulate RADAR tracks
      SimulateRadarTracks(CurSimTime);
      LastTrackTime = CurSimTime;
    }
  }
}

void UUnrealRadar::BeginPlay() { Super::BeginPlay(); }

void UUnrealRadar::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  Super::EndPlay(EndPlayReason);
  Radar.EndUpdate();
}

void UUnrealRadar::InitializePose(const projectairsim::Transform& PoseNed) {
  SetRelativeTransform(UnrealTransform::FromGlobalNed(PoseNed));

  // Check that the initial pose was set correctly
  projectairsim::Transform InitializedPose = UnrealTransform::GetPoseNed(this);
  projectairsim::Vector3 InitializedRPY = projectairsim::TransformUtils::ToDegrees(
      projectairsim::TransformUtils::ToRPY(InitializedPose.rotation_));
  UnrealLogger::Log(
      projectairsim::LogLevel::kTrace,
      TEXT("[UnrealRadar] Radar '%S': InitializePose(). "
           "RelativeLocation (%f,%f,%f) RelativeRotationRPY (%f,%f,%f)"),
      Radar.GetId().c_str(), InitializedPose.translation_.x(),
      InitializedPose.translation_.y(), InitializedPose.translation_.z(),
      InitializedRPY.x(), InitializedRPY.y(), InitializedRPY.z());
}

void UUnrealRadar::GenerateFullFOVFrame() {
  FullFOVFrame.clear();

  // Generate rectangular grid from top left to bottom right with beams evenly
  // spaced to fill FOV at each axis resolution spacing
  int ElevStepMax = static_cast<int>(Settings.fov_elevation_max /
                                     Settings.fov_elevation_resolution);
  int ElevStepMin = static_cast<int>(Settings.fov_elevation_min /
                                     Settings.fov_elevation_resolution);

  int AzimStepMax = static_cast<int>(Settings.fov_azimuth_max /
                                     Settings.fov_azimuth_resolution);
  int AzimStepMin = static_cast<int>(Settings.fov_azimuth_min /
                                     Settings.fov_azimuth_resolution);

  // top -> bottom
  for (int IdxElev = ElevStepMax; IdxElev >= ElevStepMin; --IdxElev) {
    // left -> right
    for (int IdxAzim = AzimStepMin; IdxAzim <= AzimStepMax; ++IdxAzim) {
      const float CurElev = IdxElev * Settings.fov_elevation_resolution;
      const float CurAzim = IdxAzim * Settings.fov_azimuth_resolution;

      // Save the subset of masks that cover this beam's azimuth/elevation, to
      // reduce the number of masks needed to check against each beam
      // detection's range/velocity/rcs later.
      std::vector<projectairsim::RadarMask> BeamMasks;
      for (const auto& CurMask : Settings.masks) {
        if (CurAzim >= CurMask.azimuth_min && CurAzim <= CurMask.azimuth_max &&
            CurElev >= CurMask.elevation_min &&
            CurElev <= CurMask.elevation_max) {
          BeamMasks.push_back(CurMask);
        }
      }

      FullFOVFrame.emplace_back(CurAzim, CurElev, BeamMasks);
    }
  }
}

RadarBeamPoints UUnrealRadar::GenerateContinuousBeams(int points_per_frame) {
  RadarBeamPoints beams;
  if (points_per_frame <= 0) {
    return beams;
  }

  beams.reserve(static_cast<size_t>(points_per_frame));

  const float az_min = Settings.fov_azimuth_min;
  const float az_max = Settings.fov_azimuth_max;
  const float el_min = Settings.fov_elevation_min;
  const float el_max = Settings.fov_elevation_max;
  const float az_span = az_max - az_min;

  int az_bins = 1;
  if (Settings.fov_azimuth_resolution > 0.0f && az_span > 0.0f) {
    az_bins = static_cast<int>(std::floor(az_span /
                                          Settings.fov_azimuth_resolution)) + 1;
    az_bins = std::max(az_bins, 1);
  }

  const int base_count = points_per_frame / az_bins;
  const int remainder = points_per_frame % az_bins;

  for (int bin_idx = 0; bin_idx < az_bins; ++bin_idx) {
    const int bin_count = base_count + (bin_idx < remainder ? 1 : 0);
    if (bin_count <= 0) {
      continue;
    }

    float bin_start = az_min;
    float bin_end = az_max;
    if (Settings.fov_azimuth_resolution > 0.0f && az_bins > 1) {
      bin_start = az_min + bin_idx * Settings.fov_azimuth_resolution;
      bin_end = bin_start + Settings.fov_azimuth_resolution;
      if (bin_end > az_max || bin_idx == az_bins - 1) {
        bin_end = az_max;
      }
    }

    for (int i = 0; i < bin_count; ++i) {
      const float azim = SampleUniform(bin_start, bin_end);
      const float elev = SampleUniform(el_min, el_max);

      std::vector<projectairsim::RadarMask> beam_masks;
      for (const auto& cur_mask : Settings.masks) {
        if (azim >= cur_mask.azimuth_min && azim <= cur_mask.azimuth_max &&
            elev >= cur_mask.elevation_min &&
            elev <= cur_mask.elevation_max) {
          beam_masks.push_back(cur_mask);
        }
      }

      beams.emplace_back(azim, elev, beam_masks);
    }
  }

  return beams;
}

RadarBeamPoints UUnrealRadar::GenerateBeamsToShoot(TimeNano SimTime) {
  if (!Settings.continuous_coverage) {
    // Use the full FOV frame sweep of beam points for every round
    return FullFOVFrame;
  }

  int points_per_frame = Settings.points_per_frame;
  if (points_per_frame <= 0) {
    points_per_frame = static_cast<int>(FullFOVFrame.size());
  }
  if (Settings.max_points_per_frame > 0 &&
      points_per_frame > Settings.max_points_per_frame) {
    points_per_frame = Settings.max_points_per_frame;
  }

  return GenerateContinuousBeams(points_per_frame);
}

// Simulate shooting a radar beam via Unreal line-tracing.
FHitResult UUnrealRadar::ShootSingleBeam(const FVector& RadarBodyLoc,
                                         const FRotator& RadarBodyRot,
                                         const RadarBeamPoint& BeamPoint) {
  // Calculate direction vector along which ray will be shot
  // Positive elevation is up, positive azimuth is right in UE coords
  FRotator BeamRotInSensorFrame(
      projectairsim::TransformUtils::ToDegrees(BeamPoint.elevation) /*pitch*/,
      projectairsim::TransformUtils::ToDegrees(BeamPoint.azimuth) /*yaw*/,
      0 /*roll*/);

  FRotator BeamRotInWorldFrame =
      UKismetMathLibrary::ComposeRotators(BeamRotInSensorFrame, RadarBodyRot);

  FVector RayDirectionVector =
      UKismetMathLibrary::GetForwardVector(BeamRotInWorldFrame);

  // calculate "EndTrace": point corresponding to end of line trace.
  FVector EndTrace =
      RadarBodyLoc +
      (projectairsim::TransformUtils::ToCentimeters(Settings.range_max) *
       RayDirectionVector);

  // Shoot ray via LineTraceSingleByChannel, result is saved in HitInfo
  FCollisionQueryParams TraceParams;
  TraceParams.AddIgnoredActor(OwnerActor);  // don't hit yourself
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

  FHitResult HitInfo(ForceInit);

  const float BeamRadiusCm =
      projectairsim::TransformUtils::ToCentimeters(Settings.beam_radius);
  if (BeamRadiusCm > 0.0f) {
    const FCollisionShape BeamShape =
        FCollisionShape::MakeSphere(BeamRadiusCm);
    UnrealWorld->SweepSingleByChannel(
        HitInfo, RadarBodyLoc, EndTrace, FQuat::Identity, ECC_Visibility,
        BeamShape, TraceParams,
        FCollisionResponseParams::DefaultResponseParam);
  } else {
    UnrealWorld->LineTraceSingleByChannel(
        HitInfo, RadarBodyLoc, EndTrace, ECC_Visibility, TraceParams,
        FCollisionResponseParams::DefaultResponseParam);
  }

  return HitInfo;
}

void UUnrealRadar::SimulateRadarDetections(const TimeNano SimTime) {
  // 1. Decide list of beams to shoot for this tick
  RadarBeamPoints BeamsToShoot = GenerateBeamsToShoot(SimTime);

  // 2. Shoot beams to get detection hits and process RADAR Detections
  FVector RadarBodyLoc = GetComponentLocation();
  FRotator RadarBodyRot = GetComponentRotation();

  RadarDetections RawDetections;
  TArray<FHitResult> RawGroundTruthHits;
  FCriticalSection Mutex;

  ParallelFor(BeamsToShoot.size(), [&](int32 Idx) {
    const projectairsim::RadarBeamPoint& Beam = BeamsToShoot[Idx];
    const FHitResult HitInfo =
        ShootSingleBeam(RadarBodyLoc, RadarBodyRot, Beam);

    AActor* HitActor = HitInfo.GetActor();

    if (HitInfo.bBlockingHit == false || HitActor == nullptr) {
      // For now, only keep hits that are blocking and have an associated actor
      // to be able to get ground truth data from. In the future, maybe this
      // could be generalized to handle hits at the component level or with
      // multiple non-blocking hits along the line trace.
      return;
    }

    RadarDetection Detection;
    Detection.elevation = Beam.elevation;
    Detection.azimuth = Beam.azimuth;

    FVector RadarToHitPoint = HitInfo.ImpactPoint - RadarBodyLoc;
    Detection.range =
        projectairsim::TransformUtils::ToMeters(RadarToHitPoint.Size());

    // Get velocity from hit actor and calculate relative velocity along
    // the RadarToHitPoint beam vector.
    const projectairsim::Vector3 SensorVel = Radar.GetSensorVelocity();
    projectairsim::Kinematics Kin = GetKinematicsFromActor(HitActor);
    const projectairsim::Vector3 RelativeVel = Kin.twist.linear - SensorVel;
    const projectairsim::Vector3 Dir = UnrealHelpers::ToVector3(RadarToHitPoint);
    Detection.velocity = RelativeVel.dot(Dir.normalized());
    Detection.rcs_sqm = EstimateRadarCrossSection(HitActor);

    // Check if detection should be masked out. Beam.beam_masks are the masks
    // that have this beam's azimuth and elevation within them, processed by
    // UUnrealRadar::GenerateFullFOVFrame().
    for (const projectairsim::RadarMask& CurMask : Beam.beam_masks) {
      bool bMaskByRange = Detection.range >= CurMask.range_min &&
                          Detection.range <= CurMask.range_max;
      bool bMaskByVelocity = Detection.velocity >= CurMask.velocity_min &&
                             Detection.velocity <= CurMask.velocity_max;
      bool bMaskByRCS = Detection.rcs_sqm >= CurMask.rcs_sqm_min &&
                        Detection.rcs_sqm <= CurMask.rcs_sqm_max;
      if (bMaskByRange || bMaskByVelocity || bMaskByRCS) {
        return;
      }
    }

    // Passed all masks, keep detection
    Mutex.Lock();
    RawDetections.push_back(Detection);
    RawGroundTruthHits.Add(HitInfo);
    Mutex.Unlock();
  });

  RadarDetections Detections;
  TArray<FHitResult> GroundTruthHits;
  Detections.reserve(RawDetections.size());
  GroundTruthHits.Reserve(RawGroundTruthHits.Num());

  struct BinKey {
    int azimuth_bin;
    int elevation_bin;
    int range_bin;

    bool operator==(const BinKey& other) const {
      return azimuth_bin == other.azimuth_bin &&
             elevation_bin == other.elevation_bin &&
             range_bin == other.range_bin;
    }
  };

  struct BinKeyHash {
    size_t operator()(const BinKey& key) const {
      size_t h1 = std::hash<int>{}(key.azimuth_bin);
      size_t h2 = std::hash<int>{}(key.elevation_bin);
      size_t h3 = std::hash<int>{}(key.range_bin);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  std::unordered_map<BinKey, size_t, BinKeyHash> BinToIndex;
  const bool use_clustering = Settings.cluster_by_resolution;
  const float az_res = Settings.fov_azimuth_resolution;
  const float el_res = Settings.fov_elevation_resolution;
  const float range_res = Settings.range_resolution;
  const float range_min = Settings.range_min;
  const float range_max = Settings.range_max;

  for (size_t idx = 0; idx < RawDetections.size(); ++idx) {
    RadarDetection Detection = RawDetections[idx];
    const FHitResult& HitInfo = RawGroundTruthHits[static_cast<int32>(idx)];

    if (Settings.point_dropout_probability > 0.0f &&
        SampleUniform(0.0f, 1.0f) < Settings.point_dropout_probability) {
      continue;
    }

    if (Settings.range_noise_stddev > 0.0f)
      Detection.range += SampleNormal(Settings.range_noise_stddev);
    if (Settings.azimuth_noise_stddev > 0.0f)
      Detection.azimuth += SampleNormal(Settings.azimuth_noise_stddev);
    if (Settings.elevation_noise_stddev > 0.0f)
      Detection.elevation += SampleNormal(Settings.elevation_noise_stddev);
    if (Settings.velocity_noise_stddev > 0.0f)
      Detection.velocity += SampleNormal(Settings.velocity_noise_stddev);

    if (Settings.quantize_azimuth) {
      Detection.azimuth =
          QuantizeValue(Detection.azimuth, az_res, Settings.fov_azimuth_min);
    }
    if (Settings.quantize_elevation) {
      Detection.elevation = QuantizeValue(Detection.elevation, el_res,
                                          Settings.fov_elevation_min);
    }
    if (Settings.quantize_range) {
      Detection.range = QuantizeValue(Detection.range, range_res, range_min);
    }
    if (Settings.quantize_velocity) {
      Detection.velocity = QuantizeValue(
          Detection.velocity, Settings.velocity_resolution, 0.0f);
    }

    Detection.azimuth = std::clamp(Detection.azimuth,
                                   Settings.fov_azimuth_min,
                                   Settings.fov_azimuth_max);
    Detection.elevation = std::clamp(Detection.elevation,
                                     Settings.fov_elevation_min,
                                     Settings.fov_elevation_max);
    Detection.range = std::clamp(Detection.range, range_min, range_max);
    Detection.velocity = std::clamp(Detection.velocity, -Settings.velocity_max,
                                    Settings.velocity_max);

    if (Detection.range < range_min || Detection.range > range_max) {
      continue;
    }

    if (use_clustering) {
      int az_bin = 0;
      int el_bin = 0;
      int r_bin = 0;
      if (az_res > 0.0f) {
        az_bin = static_cast<int>(
            std::floor((Detection.azimuth - Settings.fov_azimuth_min) /
                       az_res));
      }
      if (el_res > 0.0f) {
        el_bin = static_cast<int>(
            std::floor((Detection.elevation - Settings.fov_elevation_min) /
                       el_res));
      }
      if (range_res > 0.0f) {
        r_bin = static_cast<int>(
            std::floor((Detection.range - range_min) / range_res));
      }

      BinKey key{az_bin, el_bin, r_bin};
      auto itr = BinToIndex.find(key);
      if (itr == BinToIndex.end()) {
        const size_t new_index = Detections.size();
        BinToIndex.emplace(key, new_index);
        Detections.push_back(Detection);
        GroundTruthHits.Add(HitInfo);
      } else {
        const size_t existing_index = itr->second;
        if (Detection.range < Detections[existing_index].range) {
          Detections[existing_index] = Detection;
          GroundTruthHits[static_cast<int32>(existing_index)] = HitInfo;
        }
      }
    } else {
      Detections.push_back(Detection);
      GroundTruthHits.Add(HitInfo);
    }
  }

  // Optional - draw debug hit points on Unreal scene
  if (Settings.draw_debug_points && UnrealWorld) {
    for (const auto& HitInfo : GroundTruthHits) {
      DrawDebugPoint(
          UnrealWorld, HitInfo.ImpactPoint,
          10,                          // size
          FColor(0, 255, 0),           // RGB
          false,                       // persistent (never goes away)
          Settings.detection_interval  // time that point persists on object
      );
    }
  }

  // 3. Publish RADAR detection message
  auto RadarTransformStamped = UnrealTransform::GetPoseNed(this);
  projectairsim::Pose RadarPose(RadarTransformStamped.translation_,
                              RadarTransformStamped.rotation_);
  QueueDetectionMessage(SimTime, RadarPose, Detections);

  // 4. Accumulate detections until the next track update
  AccumulatedGroundTruthHits.Append(GroundTruthHits);
  AccumulatedDetections.insert(AccumulatedDetections.end(), Detections.begin(),
                               Detections.end());
}

void UUnrealRadar::SimulateRadarTracks(const TimeNano SimTime) {
  // Calculate RADAR tracks from detections
  RadarTracks CurTracks;

  // 1. Associate detections to existing tracks
  //      For ideal sensor, use hit object's name to directly link detection
  //      to track object ID. Every detection will have same kinematics data.
  //      For simulated sensor, implement something like:
  //      https://en.wikipedia.org/wiki/Radar_tracker#Plot_to_track_association

  std::unordered_map<int, int> TrackIDToDetectionIdx;

  for (int32 Idx = 0; Idx < AccumulatedDetections.size(); ++Idx) {
    check(AccumulatedDetections.size() == AccumulatedGroundTruthHits.Num());
    const RadarDetection& Detection = AccumulatedDetections[Idx];

    // Use ground truth actor name to check if an existing track ID already
    // exists for this actor.
    const FHitResult& HitInfo = AccumulatedGroundTruthHits[Idx];
    auto HitActor = HitInfo.GetActor();
    // TODO It's possible that the HitActor was from a ProcMesh tile that has
    // been swapped out in between the detection and track processing, so this
    // can become nullptr. Need to consider how to handle this case.
    if (HitActor == nullptr) {
      UnrealLogger::Log(
          projectairsim::LogLevel::kWarning,
          TEXT("[UnrealRadar::SimulateRadarTracks] HitActor became nullptr "
               "between detection and track calculations."));
      continue;
    }
    std::string ObjName(TCHAR_TO_UTF8(*(HitActor->GetName())));
    auto Itr = TrackNameToID.find(ObjName);
    if (Itr == TrackNameToID.end()) {
      // New detection
      TrackIDToDetectionIdx[NextTrackID] = Idx;

      // Initialize a set of track data for this object
      TrackIDToName[NextTrackID] = ObjName;
      TrackNameToID[ObjName] = NextTrackID;
      TrackIDToTrack[NextTrackID] = projectairsim::RadarTrack(NextTrackID);
      TrackIDToAssocCnt[NextTrackID] = 0;
      TrackIDToConfirmation[NextTrackID] = false;

      NextTrackID++;
    } else {
      // Existing detection
      TrackIDToDetectionIdx[Itr->second] = Idx;
    }
  }

  // 2. Create new tracks for unassociated detections
  //      Require associating 3 out of 5 last detections to consider a track
  //      established (M-of-N rule).

  for (auto& [TrackID, Track] : TrackIDToTrack) {
    if (TrackIDToDetectionIdx.count(TrackID) == 0) {
      TrackIDToAssocCnt[TrackID]--;
    } else {
      if (TrackIDToAssocCnt.at(TrackID) <
          UUnrealRadar::kTrackNumLastDetections) {
        TrackIDToAssocCnt[TrackID]++;
      }
      if (TrackIDToAssocCnt.at(TrackID) >= UUnrealRadar::kTrackConfirmThresh) {
        TrackIDToConfirmation[TrackID] = true;
      }
    }
  }

  // 3. Update tracks for each object using new associated detection
  //      For ideal sensor, just update the track's position/velocity/accel
  //      with the latest detection since it's basically ground-truth.
  //      For simulated sensor, implement EKF track smoothing.

  for (auto& [TrackID, Track] : TrackIDToTrack) {
    auto Itr = TrackIDToDetectionIdx.find(TrackID);
    if (Itr != TrackIDToDetectionIdx.end()) {
      // This track has a new associated data point
      const int Idx = Itr->second;
      const RadarDetection& Detection = AccumulatedDetections.at(Idx);
      const FHitResult& HitInfo = AccumulatedGroundTruthHits[Idx];
      const AActor* HitActor = HitInfo.GetActor();

      // Set estimated pos/vel/accel by actor's kinematics and convert the
      // global frame kinematics to Radar sensor's local frame
      projectairsim::Kinematics Kin = GetKinematicsFromActor(HitActor);
      FVector RadarBodyLoc = GetComponentLocation();
      FRotator RadarBodyRot = GetComponentRotation();

      FVector PosInSensorLocal = RadarBodyRot.UnrotateVector(
          UnrealTransform::NedToUnrealLinear(Kin.pose.position) - RadarBodyLoc);
      FVector VelInSensorLocal = RadarBodyRot.UnrotateVector(
          UnrealTransform::NedToUnrealLinear(Kin.twist.linear));
      FVector AccelInSensorLocal = RadarBodyRot.UnrotateVector(
          UnrealTransform::NedToUnrealLinear(Kin.accels.linear));

      Track.position_est = ConvertToSensorFrame(
          UnrealTransform::UnrealToNedLinear(PosInSensorLocal));
      Track.velocity_est = ConvertToSensorFrame(
          UnrealTransform::UnrealToNedLinear(VelInSensorLocal));
      Track.accel_est = ConvertToSensorFrame(
          UnrealTransform::UnrealToNedLinear(AccelInSensorLocal));

      Track.range_est = Track.position_est.norm();
      Track.elevation_est =
          std::asin(-Track.position_est.z() / Track.range_est);
      Track.azimuth_est =
          std::atan2(Track.position_est.y(), Track.position_est.x());

      Track.rcs_sqm = Detection.rcs_sqm;
    } else {
      // This track did not get a new associated data point, just leave it as-is
      // for now. If using EKF, this may still need an estimation update for the
      // last delta time.
    }
  }

  // 4. Delete tracks without any associated detections
  //      Delete track if not associated for last 3 detections.

  for (auto Itr = TrackIDToTrack.begin(); Itr != TrackIDToTrack.end();) {
    const auto TrackID = Itr->first;
    if (TrackIDToConfirmation[TrackID] == true &&
        TrackIDToAssocCnt[TrackID] < UUnrealRadar::kTrackDeleteThresh) {
      TrackIDToAssocCnt.erase(TrackID);
      TrackIDToConfirmation.erase(TrackID);
      std::string ObjName = TrackIDToName.at(TrackID);
      TrackNameToID.erase(ObjName);
      TrackIDToName.erase(TrackID);
      Itr = TrackIDToTrack.erase(Itr);
    } else {
      Itr++;
    }
  }

  // 5. Pack and publish RADAR track message

  for (const auto& [TrackID, Track] : TrackIDToTrack) {
    if (TrackIDToConfirmation.at(TrackID) == true) {
      CurTracks.push_back(Track);
    }
  }
  auto RadarTransformStamped = UnrealTransform::GetPoseNed(this);
  projectairsim::Pose RadarPose(RadarTransformStamped.translation_,
                              RadarTransformStamped.rotation_);
  QueueTrackMessage(SimTime, RadarPose, CurTracks);

  // Reset accumulated detection data to start batch for next track update
  AccumulatedGroundTruthHits.Empty();
  AccumulatedDetections.clear();
}

float UUnrealRadar::EstimateRadarCrossSection(const AActor* Actor) {
  if (Actor == nullptr) return 0.0f;

  // Estimate the RCS by the size of bounding sphere for the actor's components
  bool bFirstBound = true;
  FBoxSphereBounds ActorBounds;
  // Get actor's component bounds following AActor::GetActorBounds() example
  Actor->ForEachComponent<UPrimitiveComponent>(
      false, [&](const UPrimitiveComponent* InPrimComp) {
        // Only use collidable components to find collision bounding box.
        if (InPrimComp->IsRegistered() && InPrimComp->IsCollisionEnabled()) {
          if (bFirstBound) {
            ActorBounds = InPrimComp->Bounds;
            bFirstBound = false;
          } else {
            ActorBounds = ActorBounds + InPrimComp->Bounds;
          }
        }
      });

  const float RadiusMeters =
      projectairsim::TransformUtils::ToMeters(ActorBounds.SphereRadius);

  const float RCS_sqm =
      M_PI * RadiusMeters * RadiusMeters * Settings.rcs_adjust_factor;

  return RCS_sqm;
}

inline projectairsim::Vector3 UUnrealRadar::ConvertToSensorFrame(
    const projectairsim::Vector3& VecNED) {
  // // Convert NED (+X forward, +Y right, +Z down) to Echodyne Radar sensor
  // // local frame (+X left, +Y up, +Z forward)
  // return {-VecNED.y(), -VecNED.z(), VecNED.x()};

  // // Convert NED (+X forward, +Y right, +Z down) to generic Radar sensor
  // // local frame (+X forward, +Y left, +Z up)
  // return {VecNED.x(), -VecNED.y(), -VecNED.x()};

  // Passthrough as NED (+X forward, +Y right, +Z down)
  return VecNED;
}

inline projectairsim::Kinematics UUnrealRadar::GetKinematicsFromActor(
    const AActor* Actor) {
  // TODO Refactor AUnrealRobot and AUnrealEnvActor so we don't need to dynamic
  // cast to get their kinematics.
  projectairsim::Kinematics Kin;  // constructs with zero values

  const AUnrealRobot* RobotActor = Cast<const AUnrealRobot>(Actor);
  const AUnrealEnvActor* EnvActor = Cast<const AUnrealEnvActor>(Actor);

  if (RobotActor) {
    Kin = RobotActor->GetKinematics();
  } else if (EnvActor) {
    Kin = EnvActor->GetKinematics();
  } else if (Actor) {
    Kin.pose.position =
        UnrealTransform::UnrealToNedLinear(Actor->GetActorLocation());
  } else {  // Actor == nullptr
    // Just leave Kin as its initialized zero values
  }

  return Kin;
}

float UUnrealRadar::SampleNormal(float stddev) {
  if (stddev <= 0.0f) {
    return 0.0f;
  }
  std::normal_distribution<float> dist(0.0f, stddev);
  return dist(Rng);
}

float UUnrealRadar::SampleUniform(float min_val, float max_val) {
  if (min_val > max_val) {
    std::swap(min_val, max_val);
  }
  std::uniform_real_distribution<float> dist(min_val, max_val);
  return dist(Rng);
}

float UUnrealRadar::QuantizeValue(float value, float resolution, float origin) {
  if (resolution <= 0.0f) {
    return value;
  }
  const float normalized = (value - origin) / resolution;
  const float rounded = std::round(normalized);
  return origin + rounded * resolution;
}

void UUnrealRadar::FlushPendingMessages(TimeNano SimTime) {
  while (!PendingDetectionMsgs.empty() &&
         PendingDetectionMsgs.front().publish_time <= SimTime) {
    Radar.PublishRadarDetectionMsg(PendingDetectionMsgs.front().message);
    PendingDetectionMsgs.pop_front();
  }

  while (!PendingTrackMsgs.empty() &&
         PendingTrackMsgs.front().publish_time <= SimTime) {
    Radar.PublishRadarTrackMsg(PendingTrackMsgs.front().message);
    PendingTrackMsgs.pop_front();
  }
}

void UUnrealRadar::QueueDetectionMessage(
    TimeNano SimTime, const projectairsim::Pose& RadarPose,
    const std::vector<RadarDetection>& Detections) {
  projectairsim::RadarDetectionMessage DetectionMsg(SimTime, Detections,
                                                    RadarPose);
  if (DataLatency <= 0) {
    Radar.PublishRadarDetectionMsg(DetectionMsg);
    return;
  }
  PendingDetectionMsgs.emplace_back(SimTime + DataLatency, DetectionMsg);
}

void UUnrealRadar::QueueTrackMessage(
    TimeNano SimTime, const projectairsim::Pose& RadarPose,
    const std::vector<microsoft::projectairsim::RadarTrack>& Tracks) {
  projectairsim::RadarTrackMessage TrackMsg(SimTime, Tracks, RadarPose);
  if (DataLatency <= 0) {
    Radar.PublishRadarTrackMsg(TrackMsg);
    return;
  }
  PendingTrackMsgs.emplace_back(SimTime + DataLatency, TrackMsg);
}
