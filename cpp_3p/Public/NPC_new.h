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

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ExecuteNextStep(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	bool HasActiveExploreMoveTarget() const { return bIsExecutingExploreAction; }

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
		FVector LandingFootLocation = FVector::ZeroVector;
		FVector LandingActorLocation = FVector::ZeroVector;
	};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	float ProbeStepDistance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ExploreActionDuration;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.1", UIMin = "0.1"))
	float MoveInputScale;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraYawStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraPitchStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MinCameraPitchDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MaxCameraPitchDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bUpdateVisitedBeforeSampling;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bDebugDrawExploreCandidates;

private:
	void StartExploreAction();
	void ExecuteExploreAction(float DeltaTime);

	void BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildActionCandidate(ENPCExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const;
	bool IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	bool GetWorldDirectionForAction(ENPCExploreMoveAction Action, FVector& OutDirection) const;

	bool IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const;
	bool ChooseRandomCameraAction(FRotator& OutDesiredRotation) const;

private:
	bool bIsExecutingExploreAction = false;
	FVector CurrentExploreMoveTarget = FVector::ZeroVector;

	FVector CurrentExploreMoveDirection = FVector::ZeroVector;
	float CurrentExploreActionElapsed = 0.0f;

	bool bHasDesiredCameraWorldRotation = false;
	FRotator StartCameraWorldRotation = FRotator::ZeroRotator;
	FRotator DesiredCameraWorldRotation = FRotator::ZeroRotator;
};
