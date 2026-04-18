#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "NPC_new.generated.h"

class UNavigationSystemV1;

UCLASS()
class CPP_3P_API ANPC_new : public ANPC
{
	GENERATED_BODY()

public:
	ANPC_new();

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ExecuteNextStep(float DeltaTime);

protected:
	virtual void BeginPlay() override;

	struct FExploreMoveCandidate
	{
		FVector WorldDirection = FVector::ZeroVector;
		FVector NavFootLocation = FVector::ZeroVector;      // NavMesh投影出来的“脚底点”
		FVector ActorTargetLocation = FVector::ZeroVector;  // 真正用于Actor移动的“胶囊中心点”
		float VisitValue = 0.0f;
		float Weight = 0.0f;
	};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	float ProbeStepDistance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ExplorationBias;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraYawStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraPitchStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MinCameraPitchDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MaxCameraPitchDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraCollisionProbeRadius;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bUpdateVisitedBeforeSampling;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bDebugDrawExploreCandidates;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bRequireCameraCollisionFreeForMoveCandidate;

private:
	void BuildReachableMoveCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildMoveCandidate(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	float GetVisitedValueAtLocation(const FVector& WorldLocation) const;
	int32 SampleWeightedCandidateIndex(const TArray<FExploreMoveCandidate>& Candidates) const;

	bool IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const;
	bool IsCameraPoseCollisionFree(const FVector& ActorLocation, const FRotator& DesiredCameraWorldRotation) const;
	bool TryApplyRandomCameraAction();
};
