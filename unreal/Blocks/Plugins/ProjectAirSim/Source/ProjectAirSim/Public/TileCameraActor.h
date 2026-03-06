// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TileCameraActor.generated.h"

class USceneCaptureComponent2D;

UCLASS()
class PROJECTAIRSIM_API ATileCameraActor : public AActor
{
	GENERATED_BODY()

public:
	ATileCameraActor();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

	// OSM XYZ settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	int32 Zoom = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	int32 TileX = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	int32 TileY = 0;

	// Origin lat/lon corresponds to UE world (0,0)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	double OriginLat = 47.641468;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	double OriginLon = -122.140165;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	double AltitudeMeters = 122.0;

	// If true: X=North, Y=East. If false: X=East, Y=North.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bNorthIsX = false;

	// If true, uses WebMercator scale (OSM/QGC). If false, uses true ground scale (ENU).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bUseMercatorScale = true;

	// Optional local offset in cm (applied after tile mapping)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	FVector LocalOffsetCm = FVector::ZeroVector;

	// If true, interpret TileY as TMS and flip to XYZ.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bTileYIsTMS = false;

	// Apply transform at BeginPlay.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bUpdateOnBeginPlay = true;

	// Continuously update every tick.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bUpdateEveryTick = false;

	// If true, set capture component to orthographic and update its OrthoWidth.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bApplyOrthoWidthToCapture = true;

	// Optional: capture once after update.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiles")
	bool bCaptureAfterUpdate = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Tiles")
	USceneCaptureComponent2D* CaptureComponent = nullptr;

	UFUNCTION(BlueprintCallable, Category = "Tiles")
	void UpdateFromTile();
};
