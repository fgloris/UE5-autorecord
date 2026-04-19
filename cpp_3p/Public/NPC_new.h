#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "NPC_new.generated.h"

class UNavigationSystemV1;
class UCharacterMovementComponent;

UENUM()
enum class ENPCExploreMoveAction : uint8
{
	W,
	S,
	A,
	D,
	WA,
	WD,
	SA,
	SD
};

UCLASS()
class CPP_3P_API ANPC_new : public ANPC
{
	GENERATED_BODY()

public:
	ANPC_new();

	// 在 Event Tick 中持续调用：
	// - 若当前没有活动动作，则采样一个新的低探索度动作（W/A/S/D及其组合）
	// - 若当前已有活动动作，则持续执行移动动作，同时平滑更新相机
	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ExecuteNextStep(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	bool HasActiveExploreMoveTarget() const { return bHasActiveExploreMoveTarget; }

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	FVector GetCurrentExploreMoveTarget() const { return CurrentExploreMoveTarget; }

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ClearExploreMoveTarget();

	UFUNCTION(BlueprintImplementableEvent, Category = "Navigation|Explore")
	void OnExploreMoveTargetReached(const FVector& ReachedLocation);

protected:
	virtual void BeginPlay() override;

	struct FExploreMoveCandidate
	{
		ENPCExploreMoveAction Action = ENPCExploreMoveAction::W;
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "1.0", UIMin = "1.0"))
	float MoveAcceptanceRadius;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.1", UIMin = "0.1"))
	float MoveInputScale;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.1", UIMin = "0.1"))
	float ActorYawInterpSpeed;

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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore", meta = (ClampMin = "0.1", UIMin = "0.1"))
	float CameraRotationInterpSpeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bUpdateVisitedBeforeSampling;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bDebugDrawExploreCandidates;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bRequireCameraCollisionFreeForMoveCandidate;

private:
	void ExploreOneStep();
	void FollowCharacterMovement(float DeltaTime);
	void FollowCameraMovement(float DeltaTime);

	void BuildReachableMoveCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildMoveCandidateForAction(ENPCExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const;
	bool TryBuildMoveCandidate(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	bool GetWorldDirectionForAction(ENPCExploreMoveAction Action, FVector& OutDirection) const;
	float GetVisitedValueAtLocation(const FVector& WorldLocation) const;
	int32 SampleWeightedCandidateIndex(const TArray<FExploreMoveCandidate>& Candidates) const;

	bool IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const;
	bool IsCameraPoseCollisionFree(const FVector& ActorLocation, const FRotator& InDesiredCameraWorldRotation) const;
	bool ChooseRandomCameraAction(FRotator& OutDesiredRotation) const;

private:
	bool bHasActiveExploreMoveTarget = false;
	FVector CurrentExploreMoveTarget = FVector::ZeroVector;
	FVector CurrentExploreMoveNavFootLocation = FVector::ZeroVector;

	ENPCExploreMoveAction CurrentExploreMoveAction = ENPCExploreMoveAction::W;
	float CurrentExploreMoveTravelDistance = 0.0f;
	FVector CurrentExploreMoveLastLocation = FVector::ZeroVector;

	bool bHasDesiredCameraWorldRotation = false;
	FRotator DesiredCameraWorldRotation = FRotator::ZeroRotator;
};
